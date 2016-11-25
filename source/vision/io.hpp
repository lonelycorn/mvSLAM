#pragma once
#include <base/image.hpp>
#include <base/space.hpp>
#include <string>

namespace mvSLAM
{
ImageGrayscale load_image_grayscale(const std::string &filename);

bool save_image_grayscale(const std::string &filename,
                          const ImageGrayscale &image);

ImageRGB load_image_rgb(const std::string &filename);

bool save_image_rgb(const std::string &filename,
                    const ImageRGB &image);

bool load_camera_intrinsics(const std::string &filename,
                            CameraIntrinsics &intrinsics);

bool save_camera_intrinsics(const std::string &filename,
                            const CameraIntrinsics &intrinsics);
}

