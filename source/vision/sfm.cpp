#include <cassert>
#include <cstdio>
#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <vision/sfm.hpp>
#include <base/debug.hpp>

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
    VF_MATCH_INLIER_MIN = 15;
static constexpr ScalarType
    VF_MATCH_ERROR_MAX = (ScalarType) 3.0;
static constexpr ScalarType
    VF_MATCH_CONFIDENCE_LEVEL = (ScalarType) 0.99;

bool reconstruct_scene(const ImageGrayscale &image1,
                       const ImageGrayscale &image2,
                       const CameraIntrinsics &K,
                       Pose &pose2in1,
                       std::vector<Point3D> &pointsin1)
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

    // estimate fundamental matrix
    LOG("Estimating fundamental matrix.");
    std::vector<ImagePoint> image_points1 = vf1.get_image_points();
    std::vector<ImagePoint> image_points2 = vf2.get_image_points();
    cv::Mat inlier_mask;
    cv::Mat F_Mat = cv::findFundamentalMat(image_points1,
                                           image_points2,
                                           cv::FM_RANSAC,
                                           VF_MATCH_ERROR_MAX,
                                           VF_MATCH_CONFIDENCE_LEVEL,
                                           inlier_mask);
    F_Mat.convertTo(F_Mat, CV_32FC1); // NOTE: convert to 32-bit float, 1 channel
    LOG("Checking inliers.");
    size_t inlier_count = 0;
    for (int i = 0; i < inlier_mask.rows; ++i)
        inlier_count += inlier_mask.at<uchar>(i, 0);
    DEBUG("%lu inliers when computing fundamental matrix.", inlier_count);
    if (inlier_count < VF_MATCH_INLIER_MIN)
    {
        ERROR("not enough inliers when computing fundamental matrix.");
        return false;
    }
    //std::cout<<"Fundamental matrix:\n"<<F_Mat<<std::endl;

    // compute essential matrix
    LOG("Computing essential matrix.");
    cv::Mat K_Mat(3, 3, CV_32FC1);
    cv::Mat K_T_Mat(3, 3, CV_32FC1);
    K_Mat = Matrix3Type_to_Mat(K);
    cv::transpose(K_Mat, K_T_Mat);
    cv::Mat E_Mat = K_T_Mat * F_Mat * K_Mat;
    //std::cout<<"Essential matrix:\n"<<E_Mat<<std::endl;

    // recover pose by decomposing essential matrix
    LOG("Recovering relative pose.");
    ScalarType focal = (ScalarType) std::sqrt(K(0, 0) * K(1, 1));
    ImagePoint center(K(0, 2), K(1, 2));
    cv::Mat R_Mat;
    cv::Mat t_Mat;
    E_Mat.convertTo(E_Mat, CV_64FC1); // NOTE: convert to 64-bit float, 1 channel
    cv::recoverPose(E_Mat,
                    image_points1,
                    image_points2,
                    R_Mat,
                    t_Mat,
                    focal,
                    center,
                    inlier_mask); // as input: marks points that should be used
                                  // as output: marks points passed cheirality checks
    //R_Mat.convertTo(R_Mat, CV_32FC1);
    //t_Mat.convertTo(t_Mat, CV_32FC1);
    cv::Mat pose2in1_Mat(3, 4, CV_32FC1);
    R_Mat.copyTo(pose2in1_Mat(cv::Range(0, 3), cv::Range(0, 3)));
    t_Mat.copyTo(pose2in1_Mat(cv::Range(0, 3), cv::Range(3, 4)));
    //std::cout<<"pose2in1 =\n"<<pose2in1_Mat<<std::endl;

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
    std::vector<ImagePoint> inlier_image_points1;
    std::vector<ImagePoint> inlier_image_points2;
    for (int i = 0; i < inlier_mask.rows; ++i)
        if (inlier_mask.at<uchar>(i, 0) > 0)
        {
            inlier_image_points1.push_back(image_points1[i]);
            inlier_image_points2.push_back(image_points2[i]);
        }
    
    cv::Mat camera_matrix1(3, 4, CV_32FC1);
    cv::Mat camera_matrix2(3, 4, CV_32FC1);
    K_Mat.copyTo(camera_matrix1(cv::Range(0, 3), cv::Range(0, 3)));
    K_Mat.copyTo(camera_matrix2(cv::Range(0, 3), cv::Range(0, 3)));
    //std::cout<<"camera_matrix1 =\n"<<camera_matrix1<<std::endl;
    //std::cout<<"camera_matrix2 =\n"<<camera_matrix2<<std::endl;
    camera_matrix2 = K_Mat * pose2in1_Mat;
    cv::Mat points(4, inlier_count, CV_32FC1);
    cv::triangulatePoints(camera_matrix1,
                          camera_matrix2,
                          inlier_image_points1,
                          inlier_image_points2,
                          points);
    
    // prepare output
    LOG("Preparing output");
    Matrix3Type R = Mat_to_Matrix3Type(R_Mat);
    Vector3Type t = Mat_to_Vector3Type(t_Mat);
    pose2in1 = SE3(SO3(R), t);

    pointsin1.clear();
    for (size_t i = 0; i < inlier_count; ++i)
    {
        Vector3Type v = Mat_to_Vector3Type(points.col(i));
        pointsin1.emplace_back(v);
    }

    return true;
}

bool refine_scene(const VisualFeature &vf1,
                  const VisualFeature &vf2,
                  const CameraIntrinsics &K,
                  const Pose &point2in1_guess,
                  const std::vector<Point3D> pointsin1_guess,
                  PoseEstimate &pose_estimate,
                  std::vector<Point3DEstimate> &point_estimates)
{
    assert(false);
}


}
