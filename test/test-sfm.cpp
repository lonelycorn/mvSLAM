#include <base/image.hpp>
#include <vision/sfm.hpp>
#include <vision/camera.hpp>
#include <cstdio>
#include <iostream>
#include <vector>
#include "unit-test.hpp"

using namespace unit_test;

const mvSLAM::ScalarType tolerance = 0.001;

UNIT_TEST(reconstruct_scene_cube)
{
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::CameraExtrinsics P1; // by default, use identity
    mvSLAM::Vector6Type se3_2to1; // transform from camera 2 to camera 1
    se3_2to1 << 1, 0, 0, // translation in positive x
                0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P2 =
        mvSLAM::SE3::exp(se3_2to1).inverse(); // transform from world (camera 1) to camera 2

    mvSLAM::PinholeCamera c1(K, P1);
    mvSLAM::PinholeCamera c2(K, P2);

    std::vector<mvSLAM::Point3D> points_in_world_frame;
    points_in_world_frame.reserve(8);

    // generate points
#if 1
    mvSLAM::SO3 rotation(0.0, 0.0, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 1.0;
    // cube
    points_in_world_frame.emplace_back(-1, -1, -1);
    points_in_world_frame.emplace_back(-1, -1, +1);
    points_in_world_frame.emplace_back(-1, +1, -1);
    points_in_world_frame.emplace_back(-1, +1, +1);
    points_in_world_frame.emplace_back(+1, -1, -1);
    points_in_world_frame.emplace_back(+1, -1, +1);
    points_in_world_frame.emplace_back(+1, +1, -1);
    points_in_world_frame.emplace_back(+1, +1, +1);
#else
    mvSLAM::SO3 rotation(1.5, 0.7, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 0.5;
    // L-shaped rig
    points_in_world_frame.emplace_back(1, 0, 0);
    points_in_world_frame.emplace_back(0, 0, 0);
    points_in_world_frame.emplace_back(0, 2, 0);
    points_in_world_frame.emplace_back(1, 0, 3);
    points_in_world_frame.emplace_back(0, 0, 3);
    points_in_world_frame.emplace_back(0, 2, 3);
    points_in_world_frame.emplace_back(0.5, 0.0, 1.5);
    points_in_world_frame.emplace_back(0.0, 1.0, 1.5);
#endif

    // transform and scale the points
    for (auto &p : points_in_world_frame)
    {
        p = rotation * (scale * p) + translation;
    }

    // projected image points
    std::vector<mvSLAM::ImagePoint> image_points1 = c1.project_points(points_in_world_frame);
    std::vector<mvSLAM::ImagePoint> image_points2 = c2.project_points(points_in_world_frame);

    // normalize points
    std::vector<mvSLAM::NormalizedPoint> normalized_points1 = c1.normalize_points(image_points1);
    std::vector<mvSLAM::NormalizedPoint> normalized_points2 = c2.normalize_points(image_points2);

    // reconstruction
    mvSLAM::Pose pose2in1_scaled;
    std::vector<mvSLAM::Point3D> pointsin1_scaled;
    if (!reconstruct_scene(normalized_points1,
                           normalized_points2,
                           pose2in1_scaled,
                           pointsin1_scaled))
    {
        FAIL("reconstruct_scene() failed");
    }
    auto se3_2to1_recovered = pose2in1_scaled.ln();

#if 1
    // debug output
    {
        std::cout<<"====================================="<<std::endl;
        std::cout<<"Camera 2 extrinsics:\n"<<P2<<std::endl;
        std::cout<<"Camera 2 pose in camera 1 ref frame:\n"
                 <<P2.inverse()<<std::endl;
        std::cout<<"[Recovered] Camera 2 pose in camera 1 ref frame:\n"
                 <<pose2in1_scaled<<std::endl;
        std::cout<<"se3_2to1 = \n"<<se3_2to1<<std::endl;
        std::cout<<"[Recovered] se3_2to1 = \n"<<se3_2to1_recovered<<std::endl;
        std::cout<<"Points:"<<std::endl;
        for (const auto &p : points_in_world_frame)
        {
            std::cout<<p<<std::endl<<std::endl;
        }
        std::cout<<"[Recovered] Points:"<<std::endl;
        for (const auto &p : pointsin1_scaled)
        {
            std::cout<<p<<std::endl<<std::endl;;
        }
    }
#endif 
    for (size_t i = 0; i < 6; ++i)
        ASSERT_EQUAL(se3_2to1[i], se3_2to1_recovered[i], tolerance);
    ASSERT_TRUE(points_in_world_frame.size() == pointsin1_scaled.size());
    for (size_t i = 0; i < points_in_world_frame.size(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
            ASSERT_EQUAL(points_in_world_frame[i][j], pointsin1_scaled[i][j], tolerance);
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
