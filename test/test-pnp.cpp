#include <base/image.hpp>
#include <vision/pnp.hpp>
#include <vision/camera.hpp>

#include "unit-test.hpp"

#include <iostream>

using namespace unit_test;

//#define DEBUG_OUTPUT

constexpr mvSLAM::ScalarType tolerance = 0.001;


UNIT_TEST(pnp_solve_cube)
{
    /* Ground Truth */
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::Vector6Type se3; // from camera to world
    se3 << 1, 0, 0, // translation in positive x
           0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P(mvSLAM::SE3::exp(se3).inverse()); // from world to camera

    mvSLAM::PinholeCamera c(K, P);

    std::vector<mvSLAM::Point3> points;
    points.reserve(8);

    // generate points on a cube
    mvSLAM::SO3 rotation(0.0, 0.0, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 1.0;
    points.emplace_back(-1, -1, -1);
    points.emplace_back(-1, -1, +1);
    points.emplace_back(-1, +1, -1);
    points.emplace_back(-1, +1, +1);
    points.emplace_back(+1, -1, -1);
    points.emplace_back(+1, -1, +1);
    points.emplace_back(+1, +1, -1);
    points.emplace_back(+1, +1, +1);

    // transform and scale the points
    for (auto &p : points)
    {
        p = rotation * (scale * p) + translation;
    }

    // project image points
    std::vector<mvSLAM::ImagePoint> image_points = c.project_points(points);

    // normalize image points
    std::vector<mvSLAM::IdealCameraImagePoint> normalized_points = c.normalize_points(image_points);

    // PnP
    mvSLAM::Pose pose_recovered;
    if (!pnp_solve(points,
                   normalized_points,
                   pose_recovered))
    {
        FAIL("pnp_solve() failed!");
    }
#ifdef DEBUG_OUTPUT
    {
        std::cout<<"===================================="<<std::endl;
        std::cout<<"Camera pose:\n"<<P.inverse()<<std::endl;
        std::cout<<"[Recovered] Camera pose:\n"<<pose_recovered<<std::endl;
    }
#endif
    auto se3_recovered = pose_recovered.ln();
    for (size_t i = 0; i < 6; ++i)
        ASSERT_EQUAL(se3[i], se3_recovered[i], tolerance);

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
