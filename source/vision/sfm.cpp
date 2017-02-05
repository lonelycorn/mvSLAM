#include <cassert>
#include <cstdio>
#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <vision/camera.hpp>
#include <vision/estimator-RANSAC.hpp>
#include <vision/sfm.hpp>
#include <base/debug.hpp>
#include <algorithm>
#include <random>

#define DEBUG_SFM

#ifdef DEBUG_SFM
    #define ERROR(...)  MVSLAM_ERROR(__VA_ARGS__)
    #define LOG(...)    MVSLAM_LOG(__VA_ARGS__)
    #define DEBUG(...)  MVSLAM_DEBUG(__VA_ARGS__)
#else
    #define ERROR(...)
    #define LOG(...)
    #define DEBUG(...)
#endif

namespace mvSLAM
{
static constexpr int 
    VF_MATCH_SIZE_MIN = 20;
static constexpr int 
    VF_MATCH_INLIER_MIN = 8;
static constexpr ScalarType
    VF_MATCH_ERROR_MAX = (ScalarType) 1.0;
static constexpr ScalarType
    VF_MATCH_CONFIDENCE_LEVEL = (ScalarType) 0.99;
static constexpr int cv_mat_type = cv_Mat_traits<ScalarType>::DataType;

// It seems that OpenCV yields a much better result than the home-brew version
//#define USE_OPENCV

/** Find essential matrix E such that p2.T * E * p1 == 0.
 * Internally this is implemented with RANSAC.
 * Note: for normalized cameras, essential matrix is the fundamental matrix.
 */
static bool
find_essential_matrix(const std::vector<NormalizedPoint> &p1,
                      const std::vector<NormalizedPoint> &p2,
                      const ScalarType max_error_sq,
                      const ScalarType confidence_level,
                      Matrix3Type &E21,
                      std::vector<uint8_t> &inlier_mask)
{
    assert(p1.size() >= 8);
    assert(p1.size() == p2.size());
    assert(max_error_sq > epsilon);
    assert((confidence_level > epsilon) &&
           (confidence_level < static_cast<ScalarType>(1.0) - epsilon));
#ifdef USE_OPENCV
    size_t point_count = p1.size();
    std::vector<cv::Point_<ScalarType> > cvp1, cvp2;
    cvp1.reserve(point_count);
    cvp2.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        cvp1.emplace_back(p1[i][0], p1[i][1]);
        cvp2.emplace_back(p2[i][0], p2[i][1]);
    }

    inlier_mask = std::vector<uint8_t>(point_count, 1); // use all points
    cv::Mat inlier_mask_Mat(inlier_mask, false); // just a wrapper
    cv::Mat E21_Mat = cv::findEssentialMat(cvp1, cvp2,
                                           1.0, // focal
                                           cv::Point2d(0, 0), // principal point
                                           CV_RANSAC, // method
                                           confidence_level, 
                                           std::sqrt(max_error_sq), // threshold
                                           inlier_mask_Mat);
    E21 = Mat_to_Matrix3Type(E21_Mat);
    return true;
#else // USE_OPENCV
    (void) confidence_level; // not used
    
    const size_t max_iteration = 1;
    // for normalized camera pairs, the fundamental matrix is the same as
    // the essential matrix.
    FundamentalMatrixEstimatorRANSAC estimator(max_error_sq, max_iteration);
    bool result = estimator.compute(p1, p2, E21, inlier_mask);

    // An essential matrix has 2 identical singular values
    {
        Eigen::JacobiSVD<Matrix3Type> svd(E21, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const auto &U = svd.matrixU();
        const auto &V = svd.matrixV();
        const auto &s = svd.singularValues();
        Matrix3Type S(Matrix3Type::Zero());
        ScalarType value = static_cast<ScalarType>(sqrt(s[0]*s[1]));
        S(0, 0) = value;
        S(1, 1) = value;
        S(2, 2) = static_cast<ScalarType>(0);
        E21 = U * S * V.transpose();
        std::cout<<"Singular values of E:\n"<<s<<std::endl;
        std::cout<<"should be "<<value<<std::endl;
    }

    return result;
#endif // USE_OPENCV
}

/** Decompose essential matrix into a rotation and a translation.
 * there will be 4 solutions: 
 *  {R1to2_a, t1to2}, {R1to2_a, -t1to2}, {R1to2_b, t1to2}, {R1to2_b, -t1to2}
 */
static void
decompose_essential_matrix(const Matrix3Type &E21,
                           Matrix3Type &R1to2_a,
                           Matrix3Type &R1to2_b,
                           Vector3Type &t1to2)
{
    // An essential matrix can be decomposed as E = SR,
    // where S is the cross_product_matrix for the translation t,
    // and R is the rotation matrix.
    // see R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, p258
    Eigen::JacobiSVD<Matrix3Type> svd(E21, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3Type U = svd.matrixU();
    Matrix3Type V = svd.matrixV();
    std::cout<<"\n\nU =\n"<<U<<std::endl;
    std::cout<<"V =\n"<<V<<std::endl;
    if (U.determinant() < static_cast<ScalarType>(0)) // from opencv
        U *= static_cast<ScalarType>(-1);
    if (V.determinant() < static_cast<ScalarType>(0)) // from opencv
        V *= static_cast<ScalarType>(-1);
    std::cout<<"after determinant check U =\n"<<U<<std::endl;
    std::cout<<"after determinant check V =\n"<<V<<std::endl;
    Matrix3Type Z, W;
    W <<0,-1, 0,
        1, 0, 0,
        0, 0, 1;
    Z <<0, 1, 0,
       -1, 0, 0,
        0, 0, 0;
    R1to2_a = U * W * V.transpose();
    R1to2_b = U * W.transpose() * V.transpose();
    Matrix3Type S = U * Z * U.transpose();
    t1to2 << -S(1, 2), S(0, 2), -S(0, 1);
    //t1to2 << U(0, 2), U(1, 2), U(2, 2); // from opencv
}

/** Triangulate point pairs to obtain 3D coordinates.
 * 1 Point3D for each pair.
 */
static void
triangulate_points(const Matrix3Type &R1to2,
                   const Vector3Type &t1to2,
                   const std::vector<NormalizedPoint> &p1,
                   const std::vector<NormalizedPoint> &p2,
                   std::vector<Point3D> &pointsin1)
{
    assert(p1.size() == p2.size());
    assert(p1.size() > 0);

    LOG("Generating camera projection matrices.");
    const size_t point_count = p1.size();
    const Matrix4Type P1 = SE3().get_matrix();
    const Matrix4Type P2 = SE3(SO3(R1to2), t1to2).get_matrix();
    std::vector<Point3D> result;
    result.reserve(point_count);
    std::cerr<<"P1 = \n"<<P1<<std::endl;
    std::cerr<<"P2 = \n"<<P2<<std::endl;
    
    for (size_t i = 0; i < point_count; ++i)
    {
        // linearly triangulate the i-th point
        // For each point pair (x1, x2), it holds true that
        //      x1 = P1 * X
        //      x2 = P2 * X
        // where x1 is the homogeneous point on camera 1 image,
        //       P1 is the projection matrix of camera 1,
        //       x2 is the homogeneous point on camera 2 image,
        //       P2 is the projection matrix of camera 2,
        //       X is the homogeneous point in world
        // Note that x1 and P1*X are co-linear,
        //      x1.cross_product(P1*X) = 0
        //      x2.cross_product(P2*X) = 0
        // which defines a homogenous system A*X = 0
        // A is 4-by-4 with a rank of 3 (could only be determined up
        // to a scale), and the solution X is given by the singular
        // vector corresponding to the smallest singular value.
        // X is converted to inhomogeneous coordinates by scaling.
        // see p312, Multple View Geometry in Computer Vision
        const auto &x1 = p1[i];
        const auto &x2 = p2[i];

        // Construct the system A*x = 0
        Vector4Type X(Vector4Type::Zero());
        Matrix4Type A(Matrix4Type::Zero());
        A.block<1, 4>(0, 0) = x1[0] * P1.block<1, 4>(2, 0) - P1.block<1, 4>(0, 0);
        A.block<1, 4>(1, 0) = x1[1] * P1.block<1, 4>(2, 0) - P1.block<1, 4>(1, 0);
        A.block<1, 4>(2, 0) = x2[0] * P2.block<1, 4>(2, 0) - P2.block<1, 4>(0, 0);
        A.block<1, 4>(3, 0) = x2[1] * P2.block<1, 4>(2, 0) - P2.block<1, 4>(1, 0);

#if 0
        std::cerr<<"\nx1 = \n"<<x1<<std::endl;
        std::cerr<<"x2 = \n"<<x2<<std::endl;
        std::cerr<<"A = \n"<<A<<std::endl;
#endif
        
        // SVD of A
        {
            Eigen::JacobiSVD<Matrix4Type> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            X = svd.matrixV().block<1, 4>(3, 0);
            //std::cerr<<"X = \n"<<X<<std::endl;
        }

        // convert to inhomogeneous
        if (std::abs(X[3]) < tolerance) // degenerated case?
        {
            assert(false);
        }
        else
        {
            ScalarType scale = static_cast<ScalarType>(1) / X[3];
            Point3D pt;
            pt << X[0], X[1], X[2];
            pt *= scale;
            result.push_back(pt);
        }

    }
    std::swap(pointsin1, result);
}

/** Recover relative pose by decomposing the provided essential matrix;
 * Recover points by linear triangulation.
 */
static bool
recover_pose_and_points(const Matrix3Type &E21,
                        const std::vector<NormalizedPoint> &normalized_points1,
                        const std::vector<NormalizedPoint> &normalized_points2,
                        const std::vector<uint8_t> &inliers,
                        Matrix3Type &R1to2,
                        Vector3Type &t1to2,
                        std::vector<Point3D> &pointsin1,
                        std::vector<size_t> &pointsin1_indexes)
{
    /**
     * An essential matrix can be decomposed as E = SR, where S is the cross
     * product matrix for the translation t, and R is the rotation matrix.
     * see R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, p258
     * There are 4 solutions, and we need to perform triangulation and choose
     * the solution that yields positive depth (z-coordiante) for all 3D points.
     */
#ifdef USE_OPENCV
    (void) decompose_essential_matrix;
    (void) triangulate_points;

    // use OpenCV methods
    std::vector<uint8_t> points_inliers(inliers.begin(), inliers.end());
    cv::Mat points_inliers_Mat(points_inliers, false); // share data
    std::vector<cv::Point_<ScalarType> > np1, np2;
    np1.reserve(normalized_points1.size());
    np2.reserve(normalized_points2.size());
    for (size_t i = 0; i < normalized_points1.size(); ++i)
    {
        np1.emplace_back(normalized_points1[i][0], normalized_points1[i][1]);
        np2.emplace_back(normalized_points2[i][0], normalized_points2[i][1]);
    }

    LOG("Recovering camera 2 pose in camera 1 ref frame.");
    // camera 2 pose is given by the transform from camera 2 ref frame to camera 1 ref frame
    // cv::recoverPose() internally calls cv::decomposeEssentialMatrix() and cv::triangulatePoints()
    // to determine the best camera 2 project matrix [R, t]. in other worlds, R and t describe
    // the transform from camera 1 ref frame to camera 2 ref frame.
    cv::Mat R1to2_Mat, t1to2_Mat;
    cv::Mat E21_Mat = Matrix3Type_to_Mat(E21);
    int inlier_count = cv::recoverPose(E21_Mat,
                                       np1, np2,
                                       R1to2_Mat, t1to2_Mat,
                                       1.0, // focal
                                       cv::Point2d(0, 0), // principle point
                                       points_inliers_Mat);
    assert(points_inliers.size() == (size_t) inlier_count);

    LOG("Checking inliers.");
    DEBUG("%d inliers when recovering relative pose.", inlier_count);
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        ERROR("not enough inliers when recovering relative pose.");
        return false;
    }

    LOG("Triangulating points in camera 1 ref frame.");
    // camera 1 projection matrix is identity
    // camera 2 projection matrix is the transform from camera 1 ref frame to camera 2 ref frame
    cv::Mat camera_matrix1_Mat = cv::Mat::eye(3, 4, cv_mat_type);
    cv::Mat camera_matrix2_Mat = cv::Mat(3, 4, cv_mat_type);
    R1to2_Mat.copyTo(camera_matrix2_Mat(cv::Range(0, 3), cv::Range(0, 3)));
    t1to2_Mat.copyTo(camera_matrix2_Mat(cv::Range(0, 3), cv::Range(3, 4)));
    
    cv::Mat pointsin1_Mat; // 4-by-inlier_cout
    std::vector<cv::Point_<ScalarType> > inlier_np1, inlier_np2;
    inlier_np1.reserve(inlier_count);
    inlier_np2.reserve(inlier_count);
    for (size_t i = 0; i < points_inliers.size(); ++i)
        if (points_inliers[i] > 0)
        {
            inlier_np1.push_back(np1[i]);
            inlier_np2.push_back(np2[i]);
        }

    cv::triangulatePoints(camera_matrix1_Mat,
                          camera_matrix2_Mat,
                          inlier_np1,
                          inlier_np2,
                          pointsin1_Mat);
    assert(pointsin1_Mat.cols == inlier_count);

    // prepare output
    R1to2 = Mat_to_Matrix3Type(R1to2_Mat);
    t1to2 = Mat_to_Vector3Type(t1to2_Mat);
    pointsin1.clear();
    pointsin1.reserve(inlier_count);
    for (int col = 0; col < inlier_count; ++col)
    {
        Point3D p;
        for (int row = 0; row < 3; ++row)
            p[row] = pointsin1_Mat.at<ScalarType>(row, col);
        pointsin1.push_back(p);
    }
    pointsin1_indexes.reserve(inlier_count);
    for (size_t i = 0; i < points_inliers.size(); ++i)
        if (points_inliers[i] > 0)
            pointsin1_indexes.push_back(i);
#else
    std::vector<Matrix3Type> R1to2_candidates(2);
    std::vector<Vector3Type> t1to2_candidates(2);
    decompose_essential_matrix(E21,
                               R1to2_candidates[0],
                               R1to2_candidates[1],
                               t1to2_candidates[0]);
    t1to2_candidates[1] = -t1to2_candidates[0];
    pointsin1.clear();
    pointsin1_indexes.clear();
    for (const auto &R_candidate : R1to2_candidates)
    {
        for (const auto &t_candidate : t1to2_candidates)
        {
            std::vector<Point3D> points_candidate;
            triangulate_points(R_candidate,
                               t_candidate,
                               normalized_points1,
                               normalized_points2,
                               points_candidate);
            // cheirality check: all points should have positive depth (z-value)
            std::vector<size_t> points_candidate_indexes;
            for (size_t i = 0; i < points_candidate.size(); ++i)
            {
                if (points_candidate[i].z() > tolerance)
                {
                    points_candidate_indexes.push_back(i);
                }
            }

            if (points_candidate_indexes.size() > pointsin1.size()) // found a better solution
            {
                pointsin1.clear();
                pointsin1.reserve(points_candidate_indexes.size());
                for (auto idx : points_candidate_indexes)
                {
                    pointsin1.push_back(points_candidate[idx]);
                }
                pointsin1_indexes.swap(points_candidate_indexes);
                R1to2 = R_candidate;
                t1to2 = t_candidate;
            }
        }
    }
    std::cerr<<"\n\n\nBest candidate:\nR1to2=\n"<<R1to2<<"\nt1to2=\n"<<t1to2<<"point count = "<<pointsin1_indexes.size()<<std::endl;
#endif
    return true;
}

/*
bool reconstruct_scene(const ImageGrayscale &image1,
                       const ImageGrayscale &image2,
                       const CameraIntrinsics &K,
                       Pose &pose2in1_scaled,
                       std::vector<Point3D> &pointsin1_scaled)
{
    auto matched_vfs = 
        VisualFeature::match_and_filter_images(image1, image2);
    const VisualFeature &vf1 = matched_vfs.first;
    const VisualFeature &vf2 = matched_vfs.second;

    DEBUG("%lu visual features at start.", vf1.size());
    if (vf1.size() < VF_MATCH_SIZE_MIN)
    {
        ERROR("not enough visual features to start.");
        return false;
    }

    // compute normalized coordinates using inverse intrinsics
    LOG("Normalizing detected key points.");
    PinholeCamera pc(K, CameraExtrinsics());
    std::vector<NormalizedPoint> normalized_points1 =
        pc.normalize_points(vf1.get_image_points());
    std::vector<NormalizedPoint> normalized_points2 =
        pc.normalize_points(vf2.get_image_points());

    return reconstruct_scene(normalized_points1,
                             normalized_points2,
                             pose2in1_scaled,
                             pointsin1_scaled);
}
*/

bool reconstruct_scene(const std::vector<NormalizedPoint> &normalized_points1,
                       const std::vector<NormalizedPoint> &normalized_points2,
                       Pose &pose2in1_scaled,
                       std::vector<Point3D> &pointsin1_scaled)
{
    assert(normalized_points1.size() == normalized_points2.size());
    const size_t point_count = normalized_points1.size();
    (void) point_count;

    /*===============================================
     * estimate essential matrix. x2.T * E * x1 == 0 
     *===============================================*/
    LOG("Estimating essential matrix from normalized point pairs.");
    int inlier_count;
    std::vector<uint8_t> inliers;
    Matrix3Type E21;
    if (!find_essential_matrix(normalized_points1,
                               normalized_points2,
                               static_cast<ScalarType>(0.05),
                               VF_MATCH_CONFIDENCE_LEVEL,
                               E21,
                               inliers))
    {
        ERROR("Cannot find essential matrix.");
        return false;
    }

    // the more inliers the more likely this is a good essential matrix
    // but is this really necessary?
    LOG("Checking inliers.");
    inlier_count = 0;
    for (auto i : inliers)
        inlier_count += ((i > 0) ? 1 : 0);
    DEBUG("%d inliers when estimating essential matrix.", inlier_count);
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        ERROR("not enough inliers when estimating essential matrix.");
        return false;
    }

    /*==================================================
     * recover pose by decomposing the essential matrix
     * recover 3D triangulate points
     *==================================================*/
    LOG("Recovering relative pose and triangulating 3D points.");
    std::vector<size_t> point_indexes;
    std::vector<Point3D> pointsin1;
    Matrix3Type R1to2;
    Vector3Type t1to2;
    if (!recover_pose_and_points(E21,
                                 normalized_points1,
                                 normalized_points2,
                                 inliers,
                                 R1to2,
                                 t1to2,
                                 pointsin1,
                                 point_indexes))
    {
        ERROR("Cannot recover pose and points.");
        return false;
    }

#if 0
    // debugging output
    {
        std::cout<<"Essential matrix (estimated):\n"<<E21<<std::endl;
        std::cout<<"R1to2 =\n"<<R1to2<<std::endl;
        std::cout<<"t1to2 =\n"<<t1to2<<std::endl;
        std::cout<<"pointsin1 =\n";
        for (const auto &p : pointsin1)
            std::cout<<p<<std::endl<<std::endl;
    }
#endif

    /*================
     * prepare output 
     *================*/
    // the transform from camera 1 to camera 2 gives the 
    // pose of camera 2 expressed in camera 1 ref frame
    pose2in1_scaled = SE3(SO3(R1to2), t1to2).inverse();
    pointsin1_scaled.swap(pointsin1);
    return true;
}

bool refine_scene(const VisualFeature &vf1,
                  const VisualFeature &vf2,
                  const CameraIntrinsics &K,
                  const Pose &pose2to1_guess,
                  const std::vector<Point3D> pointsin1_guess,
                  PoseEstimate &pose_estimate,
                  std::vector<Point3DEstimate> &point_estimates)
{
    assert(false);
}

}
