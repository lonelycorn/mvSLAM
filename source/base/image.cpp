#include <base/image.hpp>

namespace mvSLAM
{
bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return (kp1.hash() == kp2.hash());
}

bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return !operator==(kp1, kp2);
}

}
