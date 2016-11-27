#pragma once
#include <base/opencv.hpp>
#include <base/math.hpp>
#include <vector>

namespace mvSLAM
{

using CameraExtrinsics = SE3;
using CameraIntrinsics = Matrix3Type;
using ImageRGB = cv::Mat;
using ImageGrayscale = cv::Mat;

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

using ImagePoint = cv::Point_<ScalarType>; // for points on the real camera image
using NormalizedPoint = Vector3Type; // for points in the normalized camera
using HomogeneousImagePoint = Vector3Type;
using HomogeneousNormalizedPoint = Vector4Type;

NormalizedPoint
ImagePoint_to_NormalizedPoint(const ImagePoint &ip,
                              const CameraIntrinsics &K);

std::vector<NormalizedPoint>
ImagePoint_to_NormalizedPoint(const std::vector<ImagePoint> &ips,
                              const CameraIntrinsics &K);

bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);
bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);
}
