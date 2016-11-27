#include <vision/io.hpp>
#include <fstream>
#include <iostream>
#include <cassert>
#include <opencv2/opencv.hpp>

namespace mvSLAM
{

ImageGrayscale
load_image_grayscale(const std::string &filename)
{
    ImageGrayscale image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    return image;
}

bool
save_image_grayscale(const std::string &filename,
                     const ImageGrayscale &image)
{
    return cv::imwrite(filename, image);
}

ImageRGB
load_image_rgb(const std::string &filename)
{
    ImageRGB image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    return image;
}

bool
save_image_rgb(const std::string &filename,
               const ImageRGB &image)
{
    return cv::imwrite(filename, image);
}


bool
save_camera_intrinsics(const std::string &filename,
                       const CameraIntrinsics &intrinsics)
{
    std::fstream out(filename, std::ios_base::out);
    for (size_t i = 0; i < 9; ++i)
    {
        size_t r = (i / 3);
        size_t c = (i % 3);
        out<<intrinsics(r, c);
        if (i < 9)
            out<<" ";
    }
    return bool(out);
}

bool
load_camera_intrinsics(const std::string &filename,
                       CameraIntrinsics &intrinsics)
{
    std::fstream in(filename, std::ios_base::in);
    size_t next = 0;
    ScalarType value;
    while (in >> value)
    {
        size_t r = (next / 3);
        size_t c = (next % 3);
        intrinsics(r, c) = value;
        ++next;
    }
    return (next == 9);
}

}
