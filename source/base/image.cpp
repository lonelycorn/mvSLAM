#include <base/image.hpp>
#include <fstream>
#include <iostream>
#include <cassert>

namespace mvSLAM
{

ImageGrayscale
load_image_grayscale(const std::string &filename)
{
    ImageGrayscale image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    assert((image.rows > 0) && (image.cols > 0));
    return image;
}

bool
save_image_grayscale(const std::string &filename,
                     const ImageGrayscale &image)
{
    assert((image.rows > 0) && (image.cols > 0));
    return cv::imwrite(filename, image);
}

ImageRGB
load_image_rgb(const std::string &filename)
{
    ImageRGB image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    assert((image.rows > 0) && (image.cols > 0));
    return image;
}

bool
save_image_rgb(const std::string &filename,
               const ImageRGB &image)
{
    assert((image.rows > 0) && (image.cols > 0));
    return cv::imwrite(filename, image);
}

bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return (kp1.hash() == kp2.hash());
}

bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return !operator==(kp1, kp2);
}

}
