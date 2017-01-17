#include <base/image.hpp>
#include <vision/sfm.hpp>
#include <vision/camera.hpp>
#include <cstdio>
#include <iostream>
#include <vector>
#include "unit-test.hpp"

using namespace unit_test;

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

    mvSLAM::ScalarType cube_x = 0.6,
                       cube_y = 0.0,
                       cube_z = 3.0,
                       cube_half_size = 1.0;

    // generate points on a cube
    std::vector<mvSLAM::Point3D> points_in_world_frame;
    points_in_world_frame.reserve(8);
    points_in_world_frame.emplace_back(-cube_half_size+cube_x, -cube_half_size+cube_y, -cube_half_size+cube_z);
    points_in_world_frame.emplace_back(-cube_half_size+cube_x, -cube_half_size+cube_y, +cube_half_size+cube_z);
    points_in_world_frame.emplace_back(-cube_half_size+cube_x, +cube_half_size+cube_y, -cube_half_size+cube_z);
    points_in_world_frame.emplace_back(-cube_half_size+cube_x, +cube_half_size+cube_y, +cube_half_size+cube_z);
    points_in_world_frame.emplace_back(+cube_half_size+cube_x, -cube_half_size+cube_y, -cube_half_size+cube_z);
    points_in_world_frame.emplace_back(+cube_half_size+cube_x, -cube_half_size+cube_y, +cube_half_size+cube_z);
    points_in_world_frame.emplace_back(+cube_half_size+cube_x, +cube_half_size+cube_y, -cube_half_size+cube_z);
    points_in_world_frame.emplace_back(+cube_half_size+cube_x, +cube_half_size+cube_y, +cube_half_size+cube_z);

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

#if 1
    // debug output
    {
        std::cout<<"Camera 2 extrinsics:\n"<<P2<<std::endl;
        std::cout<<"Camera 2 pose in camera 1 ref frame:\n"
                 <<P2.inverse()<<std::endl;
        std::cout<<"[Recovered] Camera 2 pose in camera 1 ref frame:\n"
                 <<pose2in1_scaled<<std::endl;
        auto se3_2to1_recovered = pose2in1_scaled.ln();
        std::cout<<"se3_2to1 = \n"<<se3_2to1<<std::endl;
        std::cout<<"se3_2to1_recovered = \n"<<se3_2to1_recovered<<std::endl;
        for (size_t i = 0; i < 6; ++i)
            ASSERT_EQUAL(se3_2to1[i], se3_2to1_recovered[i], 0.01);


        std::cout<<"Original points:"<<std::endl;
        //for (const auto &p : points
    }
#endif 

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
