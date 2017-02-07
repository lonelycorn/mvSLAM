#pragma once
#include <base/opencv.hpp>
#include <base/math.hpp>
#include <vector>

namespace mvSLAM
{

using CameraExtrinsics = SE3;
using CameraIntrinsics = Matrix3Type;
using ImageRGB = cv::Mat; // 3-channel
using ImageGrayscale = cv::Mat; // 1-channel

struct VisualFeatureConfig
{
    using DetectorType = cv::ORB;
    using DetectorContainerType = cv::Ptr<DetectorType>;
    using DetectorResultType = std::vector<cv::KeyPoint>;
    using ExtractorType = cv::ORB;
    using ExtractorContainerType = cv::Ptr<ExtractorType>;
    using ExtractorResultType = cv::Mat;
    using MatcherType = cv::BFMatcher;
    enum { MatcherNormType = cv::NORM_HAMMING };
    using MatchResultType = std::vector<cv::DMatch>; // queryIdx, trainIdx, distance
};

using ImagePoint = cv::Point_<ScalarType>; // image points of a real camera
using IdealCameraImagePoint = Vector3Type; // image points of an ideal camera

bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);
bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);
}
