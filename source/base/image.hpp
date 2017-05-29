#pragma once
#include <base/opencv.hpp>
#include <vector>

namespace mvSLAM
{

using ImageRGB = cv::Mat; // 3-channel
using ImageGrayscale = cv::Mat; // 1-channel

ImageGrayscale load_image_grayscale(const std::string &filename);

bool save_image_grayscale(const std::string &filename,
                          const ImageGrayscale &image);

ImageRGB load_image_rgb(const std::string &filename);

bool save_image_rgb(const std::string &filename,
                    const ImageRGB &image);

// cv::KeyPoint contains
//  - angle:    orientation of the key point (i.e. major axis); may be unused.
//  - class_id: id of object that the key point belongs to; may be unused.
//  - octave:   level in the image pyramid where the key point is detected.
//              Roughly speaking a pixel in the k-th level represents pow(2, k)
//              pixels in the original image.
//  - pt:       (u, v) coordinate of the key point.
//  - response: intensity of the key point (higher is better); may be unused.
//  - size:     radius of the neighborhood the key point represents.
//
// cv::DMatch contains
//  - distance (float)
//  - imgIdx (int)
//  - queryIdx (int)
//  - trainIdx (int)

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
