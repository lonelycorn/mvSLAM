#include <cmath>
#include <vision/camera.hpp>
#include "unit-test.hpp"
#include <iostream>
using namespace unit_test;
using namespace std;

static constexpr mvSLAM::ScalarType tolerance = 1e-7;

UNIT_TEST(basics)
{
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::CameraExtrinsics P; // by default use identity
    mvSLAM::PinholeCamera c(K, P);
    PASS();
}

UNIT_TEST(project_point_trivial)
{
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::CameraExtrinsics P; // by default use identity
    mvSLAM::PinholeCamera c(K, P);
    mvSLAM::Point3D p_3d(1.0, 1.0, 1.0);
    mvSLAM::ImagePoint p_image = c.project_point(p_3d);
    EQUAL_TO(p_image.x, 1.0, tolerance);
    EQUAL_TO(p_image.y, 1.0, tolerance);
    PASS();
}

UNIT_TEST(project_point)
{
    mvSLAM::CameraIntrinsics K;
    K << 0.5,   0, 20,
           0, 0.5, 10,
           0,   0,  1;
    mvSLAM::Vector6Type se3; // NOTE: the corresponding SE3 is from camera to world
    se3 << 0, 0, 1, // camera translated in neg-z
           0, 0, 0.5 * 3.14159265358; // camera rotated 90 deg cw around z
    mvSLAM::CameraExtrinsics P = mvSLAM::SE3::exp(se3);
    
    mvSLAM::PinholeCamera c(K, P);
    mvSLAM::Point3D p_3d(3.0, 2.0, 1.0);
    mvSLAM::ImagePoint p_image = c.project_point(p_3d);
    EQUAL_TO(p_image.x, -0.50+20, tolerance);
    EQUAL_TO(p_image.y,  0.75+10, tolerance);
    PASS();
}

UNIT_TEST(project_points)
{
    mvSLAM::CameraIntrinsics K;
    K << 0.5,   0, 20,
           0, 0.5, 10,
           0,   0,  1;
    mvSLAM::CameraExtrinsics P; // by default use identity
    mvSLAM::PinholeCamera c(K, P);
    vector<mvSLAM::Point3D> points_3d;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <=1; ++y)
            points_3d.emplace_back(x, y, 1.0);
    vector<mvSLAM::ImagePoint> points_image = c.project_points(points_3d);
    for (size_t i = 0; i < points_image.size(); ++i)
    {
        EQUAL_TO(points_image[i].x, points_3d[i].x() * 0.5 + 20, tolerance);
        EQUAL_TO(points_image[i].y, points_3d[i].y() * 0.5 + 10, tolerance);
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
