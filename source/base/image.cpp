#include <base/image.hpp>

namespace mvSLAM
{
/// actual conversion from ImagePoint to NormalizedPoint
static NormalizedPoint
convert_ip_to_np(const ImagePoint &ip,
                 const Matrix3Type &K_inv)
{
    Vector3Type v_image; // homogeneous representation
    v_image << ip.x, 
               ip.y, 
               (ScalarType) 1.0;
    Vector3Type v_ideal = K_inv * v_image;
    return v_ideal;
}

NormalizedPoint
ImagePoint_to_NormalizedPoint(const ImagePoint &ip,
                              const CameraIntrinsics &K)
{
    Matrix3Type K_inv = K.inverse();
    return convert_ip_to_np(ip, K_inv);
}

std::vector<NormalizedPoint>
ImagePoint_to_NormalizedPoint(const std::vector<ImagePoint> &ips,
                              const CameraIntrinsics &K)
{
    Matrix3Type K_inv = K.inverse();
    std::vector<NormalizedPoint> result;
    result.reserve(ips.size());
    for (auto ip : ips)
    {
        result.push_back(convert_ip_to_np(ip, K_inv));
    }
    return result;
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
