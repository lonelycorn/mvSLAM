#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>

#include <base/space.hpp>
#include <base/debug.hpp>
#include <base/svd.hpp>
#include <vision/camera.hpp>
#include <vision/estimator-RANSAC.hpp>
#include <vision/sfm.hpp>

namespace mvSLAM
{
static Logger logger("[sfm-solve]", false);

// NOTE: value is for identity intrinsics. If not using identity,
// must scale down accordingly
static constexpr const ScalarType
    MAX_ERROR_SQ = 5e-2;
static constexpr int 
    VF_MATCH_INLIER_MIN = 8;
static constexpr ScalarType
    VF_MATCH_CONFIDENCE_LEVEL = (ScalarType) 0.99;

/** Find essential matrix E such that p2.T * E * p1 == 0.
 * Internally this is implemented with RANSAC.
 * Note: for normalized cameras, essential matrix is the fundamental matrix.
 */
static bool
find_essential_matrix(const std::vector<IdealCameraImagePoint> &p1,
                      const std::vector<IdealCameraImagePoint> &p2,
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
#ifdef USE_OPENCV_ESSENTIAL_MATRIX
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
    E21 = Mat_to_Matrix<Matrix3Type>(E21_Mat);
    return true;
#else // USE_OPENCV_ESSENTIAL_MATRIX
    (void) confidence_level; // not used
    
    const size_t max_iteration = 1;
    // for normalized camera pairs, the fundamental matrix is the same as
    // the essential matrix.
    FundamentalMatrixEstimatorRANSAC estimator(max_error_sq, max_iteration);
    bool result = estimator.compute(p1, p2, E21, inlier_mask);

    // An essential matrix has 2 identical singular values
    {
        SVD<Matrix3Type> svd(E21, Eigen::ComputeFullU | Eigen::ComputeFullV);
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
#endif // USE_OPENCV_ESSENTIAL_MATRIX
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
    logger.info("Decomposing essential matrix.");
    // An essential matrix can be decomposed as E = SR,
    // where S is the cross_product_matrix for the translation t,
    // and R is the rotation matrix.
    // see R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, p258
    SVD<Matrix3Type> svd(E21, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3Type U = svd.matrixU();
    Matrix3Type V = svd.matrixV();
    if (U.determinant() < static_cast<ScalarType>(0)) // from opencv
        U *= static_cast<ScalarType>(-1);
    if (V.determinant() < static_cast<ScalarType>(0)) // from opencv
        V *= static_cast<ScalarType>(-1);
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
 * @param [in] inlier_mask k-th element is 1 if match between p1[k] and p2[k] is an inlier.
 * @param [out] pointsin1 triangulated points for inliers, and passed Cheirality check.
 * @param [out] point_indexes original indexes for @p pointsin1
 */
static void
triangulate_points(const Matrix3Type &R1to2,
                   const Vector3Type &t1to2,
                   const std::vector<IdealCameraImagePoint> &p1,
                   const std::vector<IdealCameraImagePoint> &p2,
                   const std::vector<uint8_t> &inlier_mask,
                   std::vector<Point3> &pointsin1,
                   std::vector<size_t> &point_indexes)
{
    assert(p1.size() == p2.size());
    assert(p1.size() > 0);

    const size_t point_count = p1.size();
    std::vector<Point3> result;
    result.reserve(point_count);
    std::vector<size_t> indexes;
    indexes.reserve(point_count);

    logger.info("Triangulating points.");

    const Matrix4Type P1 = SE3().get_matrix();
    const Matrix4Type P2 = SE3(SO3(R1to2), t1to2).get_matrix();
    
    for (size_t i = 0; i < point_count; ++i)
    {
        if (inlier_mask[i] == 0) // skip outliers
        {
            continue;
        }
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

        // Construct the system A*X = 0
        Vector4Type X(Vector4Type::Zero()); // homogeneous coord of the 3D point
        Matrix4Type A(Matrix4Type::Zero());
        A.row(0) = x1[0] * P1.row(2) - P1.row(0);
        A.row(1) = x1[1] * P1.row(2) - P1.row(1);
        A.row(2) = x2[0] * P2.row(2) - P2.row(0);
        A.row(3) = x2[1] * P2.row(2) - P2.row(1);
        
        // SVD of A
        {
            SVD<Matrix4Type> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            X = svd.matrixV().col(3);
            if (svd.matrixV().determinant() < tolerance)
            {
                X = -X;
            }
        }

        // convert to inhomogeneous
        if (std::abs(X[3]) < tolerance) // degenerated case?
        {
            assert(false);
            continue;
        }
        ScalarType scale = static_cast<ScalarType>(1) / X[3];
        Point3 pt;
        pt << X[0], X[1], X[2];
        pt *= scale;
        if (pt[2] < tolerance) // behind camera 1
        {
            continue;
        }
        Point3 pt2 = R1to2 * pt + t1to2;
        if (pt2[2] < tolerance) // behind camera 2
        {
            continue;
        }
        result.push_back(pt);
        indexes.push_back(i);

    }
    std::swap(pointsin1, result);
    std::swap(point_indexes, indexes);
}

/** Recover relative pose by decomposing the provided essential matrix;
 * Recover points by linear triangulation.
 */
static bool
recover_pose_and_points(const Matrix3Type &E21,
                        const std::vector<IdealCameraImagePoint> &normalized_points1,
                        const std::vector<IdealCameraImagePoint> &normalized_points2,
                        const std::vector<uint8_t> &inlier_mask,
                        Matrix3Type &R1to2,
                        Vector3Type &t1to2,
                        std::vector<Point3> &pointsin1,
                        std::vector<size_t> &pointsin1_indexes)
{
    pointsin1.clear();
    pointsin1_indexes.clear();

    // An essential matrix can be decomposed as E = SR, where S is the cross
    // product matrix for the translation t, and R is the rotation matrix.
    // see p258, Multiple View Geometry in Computer Vision.
    // There are 4 solutions, and we need to perform triangulation and choose
    // the solution that yields positive depth (z-coordiante) for all 3D points.
    std::vector<Matrix3Type> R1to2_candidates(2);
    std::vector<Vector3Type> t1to2_candidates(2);
    decompose_essential_matrix(E21,
                               R1to2_candidates[0],
                               R1to2_candidates[1],
                               t1to2_candidates[0]);
    t1to2_candidates[1] = -t1to2_candidates[0];

    bool success = false;
    for (const auto &R_candidate : R1to2_candidates)
    {
        for (const auto &t_candidate : t1to2_candidates)
        {
            std::vector<Point3> points_candidate;
            std::vector<size_t> points_candidate_indexes;
            triangulate_points(R_candidate,
                               t_candidate,
                               normalized_points1,
                               normalized_points2,
                               inlier_mask,
                               points_candidate,
                               points_candidate_indexes);

            if (points_candidate_indexes.size() > pointsin1.size()) // found a better solution
            {
                success = true;
                pointsin1.swap(points_candidate);
                pointsin1_indexes.swap(points_candidate_indexes);
                R1to2 = R_candidate;
                t1to2 = t_candidate;
            }
        }
    }
    return success;
}
bool sfm_solve(const std::vector<ImagePoint> &p1_,
               const std::vector<ImagePoint> &p2_,
               const CameraIntrinsics &K,
               Transformation &pose2in1_scaled,
               std::vector<Point3> &pointsin1_scaled,
               std::vector<size_t> &point_indexes_)
{
    assert(p1_.size() == p2_.size());
    const size_t point_count = p1_.size();
    logger.info(point_count, " point pairs");

    /*==================
     * normalize points
     *==================*/
    logger.info("Converting to ideal camera points");
    PinholeCamera c(K, CameraExtrinsics()); // we don't care about extrinsics
    std::vector<IdealCameraImagePoint> p1 = c.normalize_points(p1_);
    std::vector<IdealCameraImagePoint> p2 = c.normalize_points(p2_);

    /*===============================================
     * estimate essential matrix. x2.T * E * x1 == 0 
     *===============================================*/
    logger.info("Estimating essential matrix using point pairs from ideal cameras.");
    int inlier_count;
    std::vector<uint8_t> inliers;
    Matrix3Type E21;
    ScalarType max_error_sq = MAX_ERROR_SQ / K(0, 0) / K(1, 1); // geometrical mean
    if (!find_essential_matrix(p1,
                               p2,
                               max_error_sq,
                               VF_MATCH_CONFIDENCE_LEVEL,
                               E21,
                               inliers))
    {
        logger.error("Cannot find essential matrix.");
        return false;
    }

    // the more inliers the more likely this is a good essential matrix
    // but is this really necessary?
    logger.info("Checking inliers.");
    inlier_count = 0;
    for (auto i : inliers)
        inlier_count += ((i > 0) ? 1 : 0);
    logger.debug(inlier_count, " inliers when estimating essential matrix.");
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        logger.error("not enough inliers when estimating essential matrix.");
        return false;
    }

    /*==================================================
     * recover pose by decomposing the essential matrix
     * recover 3D triangulate points
     *==================================================*/
    logger.info("Recovering relative pose and triangulating 3D points.");
    std::vector<size_t> point_indexes;
    std::vector<Point3> pointsin1;
    Matrix3Type R1to2;
    Vector3Type t1to2;
    if (!recover_pose_and_points(E21,
                                 p1,
                                 p2,
                                 inliers,
                                 R1to2,
                                 t1to2,
                                 pointsin1,
                                 point_indexes))
    {
        logger.error("Cannot recover pose and points.");
        return false;
    }
    logger.debug(point_indexes.size(), " points recovered.");

    /*================
     * prepare output 
     *================*/
    // the transform from camera 1 to camera 2 gives the 
    // pose of camera 2 expressed in camera 1 ref frame
    pose2in1_scaled = SE3(SO3(R1to2), t1to2).inverse();
    pointsin1_scaled.swap(pointsin1);
    point_indexes_.swap(point_indexes);
    return true;
}

void sfm_triangulate(const std::vector<ImagePoint> &p1_,
                     const std::vector<ImagePoint> &p2_,
                     const CameraIntrinsics &K,
                     const Transformation &pose1,
                     const Transformation &pose2,
                     std::vector<Point3> &points,
                     std::vector<size_t> &point_indexes)
{
    assert(p1_.size() == p2_.size());
    size_t point_count = p1_.size();

    Transformation T_1_to_2 = pose2.inverse() * pose1;
    PinholeCamera c(K, CameraExtrinsics()); // we don't care about extrinsics
    std::vector<IdealCameraImagePoint> p1 = c.normalize_points(p1_);
    std::vector<IdealCameraImagePoint> p2 = c.normalize_points(p2_);
    const std::vector<uint8_t> inlier_mask(point_count, 1);

    triangulate_points(T_1_to_2.rotation().get_matrix(),
                       T_1_to_2.translation(),
                       p1,
                       p2,
                       inlier_mask,
                       points,
                       point_indexes);
}

}
