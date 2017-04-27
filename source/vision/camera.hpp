#pragma once
#include <base/math.hpp>
#include <base/image.hpp>
#include <base/space.hpp>
#include <vector>
namespace mvSLAM
{
using CameraExtrinsics = SE3;
using CameraIntrinsics = Matrix3Type;

class PinholeCamera
{
public:
    PinholeCamera(const std::string &fileame);
    PinholeCamera(const CameraIntrinsics &K,
                  const CameraExtrinsics &P);
    ~PinholeCamera();

    PinholeCamera(const PinholeCamera &) = default;
    PinholeCamera(PinholeCamera &&) = default;
    PinholeCamera &operator=(const PinholeCamera &) = default;
    PinholeCamera &operator=(PinholeCamera &&) = default;

    /** Project the 3D points onto the image plane.
     */
    std::vector<ImagePoint>
        project_points(const std::vector<Point3> &cartesian_points) const;
    ImagePoint
        project_point(const Point3 &cartesian_point) const;

    /** Convert the image points to those of an ideal camera.
     * An ideal camera has the identity matrix as the intrinsics.
     */
    std::vector<IdealCameraImagePoint>
        normalize_points(const std::vector<ImagePoint> &image_points) const;
    IdealCameraImagePoint
        normalize_point(const ImagePoint &image_point) const;

    const CameraIntrinsics &get_intrinsics() const;
    const CameraExtrinsics &get_extrinsics() const;

    // IO
    bool load_from_file(const std::string &filename);
    bool save_to_file(const std::string &filename) const;

private:
    CameraIntrinsics K;
    Matrix3Type K_inv; // cached for speed
    CameraExtrinsics P; // transform from world ref frame to camera ref frame.
};

}
