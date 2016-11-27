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
static constexpr size_t
    VF_MATCH_SIZE_MIN = 20;
static constexpr size_t
    VF_MATCH_INLIER_MIN = 8;
static constexpr ScalarType
    VF_MATCH_ERROR_MAX = (ScalarType) 1.0;
static constexpr ScalarType
    VF_MATCH_CONFIDENCE_LEVEL = (ScalarType) 0.99;
static constexpr int cv_mat_type = cv_Mat_traits<ScalarType>::DataType;

/** Find essential matrix E such that p2.T * E * p1 == 0.
 * Internally this is implemented with RANSAC.
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
}

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

static void
recover_pose_a_solution(const Matrix3Type &E21,
                        Matrix3Type &R1to2,
                        Vector3Type &t1to2)
{
    Matrix3Type R1to2_other;
    decompose_essential_matrix(E21, R1to2_other, R1to2, t1to2);
}

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

bool reconstruct_scene(const std::vector<NormalizedPoint> &normalized_points1,
                       const std::vector<NormalizedPoint> &normalized_points2,
                       Pose &pose2in1_scaled,
                       std::vector<Point3D> &pointsin1_scaled)
{
    assert(normalized_points1.size() == normalized_points2.size());
    const size_t point_count = normalized_points1.size();
    (void) point_count;

    // estimate essential matrix. x2.T * E * x1 == 0
    // Note: for normalized cameras, essential matrix is the fundamental matrix.
    LOG("Estimating essential matrix from normalized point pairs.");
    size_t inlier_count;
    cv::Mat inlier_mask;
    cv::Mat E_Mat;
    Matrix3Type E21;
    {
        std::vector<uint8_t> inliers;
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
        inlier_mask = cv::Mat(inliers, true);
        E_Mat = Matrix3Type_to_Mat(E21);
    }

    LOG("Checking inliers.");
    inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i)
        inlier_count += inlier_mask.at<uchar>(i, 0);
    DEBUG("%lu inliers when estimating essential matrix.", inlier_count);
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        ERROR("not enough inliers when estimating essential matrix.");
        return false;
    }
    std::cout<<"Essential matrix (estimated):\n"<<E_Mat<<std::endl;

    // use estimates;
    // recover pose by decomposing essential matrix
    LOG("Recovering relative pose.");
    cv::Mat R1to2_Mat;
    cv::Mat t1to2_Mat;
    {
        Matrix3Type R1to2;
        Vector3Type t1to2;
        recover_pose_a_solution(E21, R1to2, t1to2);
        R1to2_Mat = Matrix3Type_to_Mat(R1to2);
        t1to2_Mat = Vector3Type_to_Mat(t1to2);
        R1to2_Mat.convertTo(R1to2_Mat, cv_mat_type);
        t1to2_Mat.convertTo(t1to2_Mat, cv_mat_type);
    }
    std::cout<<"R1to2_Mat =\n"<<R1to2_Mat<<std::endl;
    std::cout<<"t1to2_Mat =\n"<<t1to2_Mat<<std::endl;

    LOG("Checking inliers.");
    inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i)
        inlier_count += inlier_mask.at<uchar>(i, 0);
    DEBUG("%lu inliers when recovering relative pose.", inlier_count);
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        ERROR("not enough liers when recovering pose.");
        return false;
    }

    // triangulate points
    LOG("Triangulating points.");
    cv::Mat pointsin1_Mat(4, inlier_count, cv_mat_type);
    /**
     * the 1st normalized camera matrix would be [ I 0 ],
     * the 2nd normalized camera matrix would be [ R t ]
     * a homogenous point in the camera 1 ref frame would be expressed in camera 2
     * ref frame as
     *      Pin2 = [ R t ] * Pin1
     * put another way, [R t] describes camera 1's pose in camera 2 ref frame
     *      pose1in2 = [ R t ]
     *                 [ 0 1 ]
     */
#if 1
    std::vector<ImagePoint> inlier_normalized_points1;
    std::vector<ImagePoint> inlier_normalized_points2;
    for (int i = 0; i < inlier_mask.rows; ++i)
        if (inlier_mask.at<uchar>(i, 0) > 0)
        {
            inlier_normalized_points1.emplace_back(normalized_points1[i][0],
                                                   normalized_points1[i][1]);
            inlier_normalized_points2.emplace_back(normalized_points2[i][0],
                                                   normalized_points2[i][1]);
        }
    
    cv::Mat camera_matrix1 = cv::Mat::eye(3, 4, cv_mat_type);
    cv::Mat camera_matrix2 = cv::Mat(3, 4, cv_mat_type);
    R1to2_Mat.copyTo(camera_matrix2(cv::Range(0, 3), cv::Range(0, 3)));
    t1to2_Mat.copyTo(camera_matrix2(cv::Range(0, 3), cv::Range(3, 4)));
    std::cout<<"camera_matrix1 =\n"<<camera_matrix1<<std::endl;
    std::cout<<"camera_matrix2 =\n"<<camera_matrix2<<std::endl;

    cv::triangulatePoints(camera_matrix1,
                          camera_matrix2,
                          inlier_normalized_points1,
                          inlier_normalized_points2,
                          pointsin1_Mat);
#endif
    // prepare output
    LOG("Preparing output");
    Matrix3Type R1to2 = Mat_to_Matrix3Type(R1to2_Mat);
    Vector3Type t1to2 = Mat_to_Vector3Type(t1to2_Mat);
    std::cout<<"R1to2 =\n"<<R1to2<<std::endl;
    std::cout<<"t1to2 =\n"<<t1to2<<std::endl;
    pose2in1_scaled = SE3(SO3(R1to2), t1to2).inverse();

    pointsin1_scaled.clear();
    pointsin1_scaled.reserve(inlier_count);
    for (size_t i = 0; i < inlier_count; ++i)
    {
        Vector3Type pin1 = Mat_to_Vector3Type(pointsin1_Mat.col(i));
        //std::cout<<"pin1 ="<<pin1.transpose()<<std::endl;
        pointsin1_scaled.emplace_back(pin1);
    }

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
