#include <base/debug.hpp>
#include <base/math.hpp>
#include <base/opencv.hpp>
#include <vision/pnp.hpp>

// FIXME: implement with Perspective-3-Points and RANSAC
#define SOLVE_PNP_RANSAC

namespace mvSLAM
{

static Logger logger("[pnp-solve]", false);

constexpr size_t
    PNP_MIN_POINT_COUNT = 7;

bool pnp_solve(const std::vector<Point3> &world_points,
               const std::vector<ImagePoint> &image_points,
               const CameraIntrinsics &K,
               Transformation &pose,
               std::vector<size_t> &inlier_point_indexes)
{
    assert(world_points.size() >= PNP_MIN_POINT_COUNT);
    assert(world_points.size() == image_points.size());

    logger.info("Solving PnP.");
    const size_t point_count = world_points.size();

    std::vector<cv::Point3_<ScalarType> > objectPoints;
    objectPoints.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto &p = world_points[i];
        objectPoints.emplace_back(p[0], p[1], p[2]);
    }
    const std::vector<cv::Point_<ScalarType> > &imagePoints = image_points;
    cv::Mat cameraMatrix = Matrix_to_Mat<Matrix3Type>(K);
    cv::Mat distCoeffs; // Null: already rectified.
    cv::Mat rvec; // output rotation vector, world to camera
    cv::Mat tvec; // output translation vector, world to camera
    bool useExtrinsicGuess = false;
    //int flags = cv::SOLVEPNP_EPNP;
    //int flags = cv::SOLVEPNP_ITERATIVE;
    int flags = cv::SOLVEPNP_P3P;
    
    // solve with OpenCV's P3P
#ifdef SOLVE_PNP_RANSAC
    int iterationsCount = 100;
    float reprojectionError = 0.05;
    float confidence = 0.95;
    // stupid openCV doesn't know how to convert size_t to other types (e.g.
    // float), and we have to convert it manually.
    std::vector<int> inlier_indexes;
    if (!cv::solvePnPRansac(objectPoints,
                            imagePoints,
                            cameraMatrix,
                            distCoeffs,
                            rvec,
                            tvec,
                            useExtrinsicGuess,
                            iterationsCount,
                            reprojectionError,
                            confidence,
                            inlier_indexes,
                            flags))
    {
        logger.info("Cannot find camera pose.");
        return false;
    }
    inlier_point_indexes.reserve(inlier_indexes.size());
    for (auto i : inlier_indexes)
    {
        inlier_point_indexes.push_back(i);
    }
    logger.info("PnP Ransac iteration count = ", iterationsCount, ", reprojectionError = ",
            reprojectionError, ", confidence = ", confidence, ", inliers count = ", inlier_point_indexes.size());
#else // SOLVE_PNP_RANSAC
    // assuming all the world-image correspondences are reliable
    if (!cv::solvePnP(objectPoints,
                      imagePoints,
                      cameraMatrix,
                      distCoeffs,
                      rvec,
                      tvec,
                      useExtrinsicGuess,
                      flags))
    {
        logger.info("Cannot find camera pose.");
        return false;
    }
    inlier_point_indexes.clear();
    inlier_point_indexes.reserve(world_points.size());
    for (auto i = 0; i < world_points.size(); ++i)
    {
        inlier_point_indexes.push_back(i);
    }
#endif // SOLVE_PNP_RANSAC

    // convert results to mvSLAM types
    SO3 R = SO3::exp(Mat_to_Matrix<Vector3Type>(rvec));
    Vector3Type t = Mat_to_Matrix<Vector3Type>(tvec);
    pose = SE3(R, t).inverse(); // camera to world

    return true;
}
}
