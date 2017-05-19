#include <vision/camera.hpp>
#include <iostream>
#include <fstream>
#include <cassert>

namespace mvSLAM
{
PinholeCamera::PinholeCamera(const std::string &filename)
{
    bool result = load_from_file(filename);
    assert(result);
}

PinholeCamera::PinholeCamera(const CameraIntrinsics &K_,
                             const CameraExtrinsics &P_):
    K(K_), K_inv(K_.inverse()), P(P_), P_inv(P_.inverse())
{
}

PinholeCamera::~PinholeCamera()
{
}

ImagePoint
PinholeCamera::project_point(const Point3 &point_in_world_frame) const
{
    Vector3Type p_in_camera_frame = P * point_in_world_frame;
    assert(p_in_camera_frame[2] > 0); // must be in front of the camera

    Vector3Type p_normalized;
    p_normalized << p_in_camera_frame[0] / p_in_camera_frame[2],
                    p_in_camera_frame[1] / p_in_camera_frame[2],
                    ScalarType(1);

    Vector3Type p_raw = K * p_normalized;
    ImagePoint result(p_raw[0], p_raw[1]);

    return result;
}

std::vector<ImagePoint>
PinholeCamera::project_points(const std::vector<Point3> &points_in_world_frame) const
{
    std::vector<ImagePoint> result;
    result.reserve(points_in_world_frame.size());

    for (const auto &point_in_world_frame : points_in_world_frame)
    {
        result.push_back(project_point(point_in_world_frame));
    }

    return result;
}

IdealCameraImagePoint
PinholeCamera::normalize_point(const ImagePoint &image_point) const
{
    Vector3Type v_image;
    v_image << image_point.x,
               image_point.y,
               (ScalarType) 1.0;
    Vector3Type v_ideal = K_inv * v_image;
    return v_ideal;

}

std::vector<IdealCameraImagePoint>
PinholeCamera::normalize_points(const std::vector<ImagePoint> &image_points) const
{
    std::vector<IdealCameraImagePoint> result;
    result.reserve(image_points.size());

    for (const auto &image_point : image_points)
    {
        result.push_back(normalize_point(image_point));
    }

    return result;
}

const CameraIntrinsics &
PinholeCamera::get_intrinsics() const
{
    return K;
}

const Matrix3Type &
PinholeCamera::get_intrinsics_inverse() const
{
    return K_inv;
}

const CameraExtrinsics &
PinholeCamera::get_extrinsics() const
{
    return P;
}

const SE3 &
PinholeCamera::get_extrinsics_inverse() const
{
    return P_inv;
}

bool
PinholeCamera::load_from_file(const std::string &filename)
{
    std::fstream in(filename, std::ios_base::in);
    // fx, fy, shear, px, py
    in >> K(0, 0) >> K(1, 1) >> K(0, 1)
       >> K(0, 2) >> K(1, 2);
    K(2, 2) = ScalarType(1);
    K_inv = K.inverse();

    // se3
    Vector6Type se3;
    for (int i = 0; i < 6; ++i)
        in >> se3[i];
    P = SE3::exp(se3);
    P_inv = P.inverse();

    in.close();
    return bool(in);
}

bool
PinholeCamera::save_to_file(const std::string &filename) const
{
    std::fstream out(filename, std::ios_base::out);

    // fx, fy, shear, px, py
    out << K(0, 0) << " " << K(1, 1) << " " << K(0, 1)
        << " " << K(0, 2) << " " << K(1, 2) << std::endl;

    // se3
    Vector6Type se3 = P.ln();
    out << se3[0] << " " << se3[1] << " " << se3[2] << " "
        << se3[3] << " " << se3[4] << " " << se3[5] << std::endl;

    out.close();
    return bool(out);
}

};
