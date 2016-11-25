#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <base/math.hpp>
#include <vector>

namespace mvSLAM
{

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

using ImagePoint = cv::Point2f;
using IdealImagePoint = cv::Point2f;

bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);
bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2);

Matrix3Type Mat_to_Matrix3Type(const cv::Mat &m);
cv::Mat Matrix3Type_to_Mat(const Matrix3Type &m);

Vector3Type Mat_to_Vector3Type(const cv::Mat &v);
cv::Mat Vector3Type_to_Mat(const Vector3Type &v);

}
