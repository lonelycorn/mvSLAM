#include <base/math.hpp>
#include <base/debug.hpp>
#include <base/opencv.hpp>
#include <vision/pnp.hpp>

#define DEBUG_PNP

#ifdef DEBUG_PNP
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
constexpr size_t
    PNP_MIN_POINT_COUNT = 4;

bool pnp_solve(const std::vector<Point3> &world_points,
               const std::vector<IdealCameraImagePoint> &image_points,
               Transformation &pose)
{
    assert(world_points.size() >= PNP_MIN_POINT_COUNT);
    assert(world_points.size() == image_points.size());

    LOG("Solving PnP.");
    const size_t point_count = world_points.size();
    constexpr int cv_Mat_type = cv_Mat_traits<ScalarType>::DataType;
    // FIXME: implement with Perspective-3-Points and RANSAC
    std::vector<cv::Point3_<ScalarType> > objectPoints;
    std::vector<cv::Point_<ScalarType> > imagePoints;
    cv::Mat cameraMatrix(cv::Mat::eye(3, 3, cv_Mat_type)); // Identity: already idealized.
    cv::Mat distCoeffs; // Null: already rectified.
    cv::Mat rvec; // output rotation vector, world to camera
    cv::Mat tvec; // output translation vector, world to camera
    bool useExtrinsicGuess = false;
    int flags = cv::SOLVEPNP_EPNP;

    // convert inputs to OpenCV types
    objectPoints.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto &p = world_points[i];
        objectPoints.emplace_back(p[0], p[1], p[2]);
    }
    imagePoints.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto &p = image_points[i];
        imagePoints.emplace_back(p[0], p[1]);
    }
    
    // solve with OpenCV's P3P
    if (!cv::solvePnP(objectPoints,
                      imagePoints,
                      cameraMatrix,
                      distCoeffs,
                      rvec,
                      tvec,
                      useExtrinsicGuess,
                      flags))
    {
        ERROR("Cannot find camera pose.");
        return false;
    }

    // convert results to mvSLAM types
    SO3 R = SO3::exp(Mat_to_Matrix<Vector3Type>(rvec));
    Vector3Type t = Mat_to_Matrix<Vector3Type>(tvec);
    pose = SE3(R, t).inverse(); // camera to world

    return true;
}
}
