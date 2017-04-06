#pragma once
#include <vision/camera.hpp>

namespace mvSLAM
{
class CameraManager
{
public:
    /// get THE camera. by default it is an ideal camera.
    static const PinholeCamera &get_camera();

    static void load_from_file(const std::string &filename);
    static void save_to_file(const std::string &filename);
};

}
