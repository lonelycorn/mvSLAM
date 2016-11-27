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
    mvSLAM::Vector6Type se3; // NOTE: the corresponding SE3 is from camera 2 to camera 1
    se3 << -1, 0, 0, // camera 2 translated in pos-x
            0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P2 = mvSLAM::SE3::exp(se3);
    std::cout<<"Camera 2 extrinsics:\n"<<P2<<std::endl;
    std::cout<<"position in camera 1 ref frame:\n"<<(P2.rotation().inverse()*P2.translation()*-1.0)<<std::endl;
    
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
#if 0
    std::cout<<"cube corners:\n";
    for (const auto p : points_in_world_frame)
        std::cout<<p.x()<<", "<<p.y()<<", "<<p.z()<<std::endl;
#endif

    // projected image points
    std::vector<mvSLAM::ImagePoint> image_points1 = c1.project_points(points_in_world_frame);
    std::vector<mvSLAM::ImagePoint> image_points2 = c2.project_points(points_in_world_frame);

#if 0
    std::cout<<"camera 1 projected points:\n";
    for (const auto ip : image_points1)
        std::cout<<ip.x<<", "<<ip.y<<std::endl;
    std::cout<<"camera 2 projected points:\n";
    for (const auto ip : image_points2)
        std::cout<<ip.x<<", "<<ip.y<<std::endl;
#endif

    // normalize points
    std::vector<mvSLAM::NormalizedPoint> normalized_points1 = c1.normalize_points(image_points1);
    std::vector<mvSLAM::NormalizedPoint> normalized_points2 = c2.normalize_points(image_points2);

#if 0
    std::cout<<"camera 1 normalized points:\n";
    for (const auto np : normalized_points1)
        std::cout<<np[0]<<", "<<np[1]<<", "<<np[2]<<std::endl;
    std::cout<<"camera 2 normalized points:\n";
    for (const auto np : normalized_points2)
        std::cout<<np[0]<<", "<<np[1]<<", "<<np[2]<<std::endl;
#endif

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

    std::cout<<"camera pose 2 in camera 1 ref frame:\n"<<pose2in1_scaled<<std::endl;
    //CameraExtrinsics P2_recovered = pose2in1_scaled;

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
