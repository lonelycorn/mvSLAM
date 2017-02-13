#include <base/image.hpp>
#include <vision/sfm.hpp>
#include <vision/camera.hpp>
#include <cstdio>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include "unit-test.hpp"

//#define DEBUG_OUTPUT

using namespace unit_test;

const mvSLAM::ScalarType tolerance = 0.02;

mvSLAM::ScalarType
get_gaussian(mvSLAM::ScalarType mean = 0.0,
             mvSLAM::ScalarType stddev = 1.0)
{
    static std::random_device rd;
    static std::mt19937 generator(rd());
    static std::normal_distribution<mvSLAM::ScalarType> standard_gaussian;
    return standard_gaussian(generator) * stddev + mean;
}

UNIT_TEST(reconstruct_scene_cube)
{
    /* Ground Truth */
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::CameraExtrinsics P1; // by default, use identity
    mvSLAM::Vector6Type se3_2to1; // transform from camera 2 to camera 1
    se3_2to1 << 1, 0, 0, // translation in positive x
                0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P2 =
        mvSLAM::SE3::exp(se3_2to1).inverse(); // transform from world (camera 1) to camera 2

    mvSLAM::PinholeCamera c1(K, P1);
    mvSLAM::PinholeCamera c2(K, P2);

    std::vector<mvSLAM::Point3> points_in_world_frame;
    points_in_world_frame.reserve(8);

    // generate points on a cube
    mvSLAM::SO3 rotation(0.0, 0.0, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 1.0;
    points_in_world_frame.emplace_back(-1, -1, -1);
    points_in_world_frame.emplace_back(-1, -1, +1);
    points_in_world_frame.emplace_back(-1, +1, -1);
    points_in_world_frame.emplace_back(-1, +1, +1);
    points_in_world_frame.emplace_back(+1, -1, -1);
    points_in_world_frame.emplace_back(+1, -1, +1);
    points_in_world_frame.emplace_back(+1, +1, -1);
    points_in_world_frame.emplace_back(+1, +1, +1);

    // transform and scale the points
    for (auto &p : points_in_world_frame)
    {
        p = rotation * (scale * p) + translation;
    }

    // projected image points
    std::vector<mvSLAM::ImagePoint> image_points1 = c1.project_points(points_in_world_frame);
    std::vector<mvSLAM::ImagePoint> image_points2 = c2.project_points(points_in_world_frame);

    // normalize points
    std::vector<mvSLAM::IdealCameraImagePoint> normalized_points1 = c1.normalize_points(image_points1);
    std::vector<mvSLAM::IdealCameraImagePoint> normalized_points2 = c2.normalize_points(image_points2);

    // reconstruction
    mvSLAM::Pose pose2in1_scaled;
    std::vector<mvSLAM::Point3> pointsin1_scaled;
    if (!reconstruct_scene(normalized_points1,
                           normalized_points2,
                           pose2in1_scaled,
                           pointsin1_scaled))
    {
        FAIL("reconstruct_scene() failed");
    }

#ifdef DEBUG_OUTPUT
    // debug output
    {
        std::cout<<"====================================="<<std::endl;
        std::cout<<"Camera 2 pose in camera 1 ref frame:\n"
                 <<P2.inverse()<<std::endl;
        std::cout<<"[Recovered] Camera 2 pose in camera 1 ref frame:\n"
                 <<pose2in1_scaled<<std::endl;
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

    auto se3_2to1_recovered = pose2in1_scaled.ln();
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

UNIT_TEST(refine_scene_L_shape)
{
    /* Ground Truth */
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::CameraExtrinsics P1; // by default, use identity
    mvSLAM::Vector6Type se3_2to1; // transform from camera 2 to camera 1
    se3_2to1 << 1, 0, 0, // translation in positive x
                0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P2 =
        mvSLAM::SE3::exp(se3_2to1).inverse(); // transform from world (camera 1) to camera 2

    mvSLAM::PinholeCamera c1(K, P1);
    mvSLAM::PinholeCamera c2(K, P2);

    std::vector<mvSLAM::Point3> points_in_world_frame;
    points_in_world_frame.reserve(8);

    // generate points on an L-shaped rig
    mvSLAM::SO3 rotation(1.5, 0.7, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 0.5;
    points_in_world_frame.emplace_back(1, 0, 0);
    points_in_world_frame.emplace_back(0, 0, 0);
    points_in_world_frame.emplace_back(0, 2, 0);
    points_in_world_frame.emplace_back(1, 0, 3);
    points_in_world_frame.emplace_back(0, 0, 3);
    points_in_world_frame.emplace_back(0, 2, 3);
    points_in_world_frame.emplace_back(0.5, 0.0, 1.5);
    points_in_world_frame.emplace_back(0.0, 1.0, 1.5);

    // transform and scale the points
    for (auto &p : points_in_world_frame)
    {
        p = rotation * (scale * p) + translation;
    }

    // projected image points
    std::vector<mvSLAM::ImagePoint> image_points1 = c1.project_points(points_in_world_frame);
    std::vector<mvSLAM::ImagePoint> image_points2 = c2.project_points(points_in_world_frame);

    /* Simeanlate measurements */
    std::vector<mvSLAM::Point2Estimate> p1_estimate;
    std::vector<mvSLAM::Point2Estimate> p2_estimate;
    p1_estimate.reserve(image_points1.size());
    p2_estimate.reserve(image_points2.size());

    const mvSLAM::ScalarType c_noise_stddev(5e-3);
    const mvSLAM::ScalarType c_noise_mean(0.0); // SHOULD be 0.

    for (const auto &p : image_points1)
    {
        // additive noise
        mvSLAM::Point2 mean;
        mean << p.x + get_gaussian(c_noise_mean, c_noise_stddev),
                p.y + get_gaussian(c_noise_mean, c_noise_stddev);
        mvSLAM::Point2Uncertainty covar(mvSLAM::Matrix2Type::Identity() * mvSLAM::sqr(c_noise_stddev));

        // isotropic covar
        p1_estimate.emplace_back(mean, covar);
    }

    for (const auto &p : image_points2)
    {
        // additive noise
        mvSLAM::Point2 mean;
        mean << p.x + get_gaussian(c_noise_mean, c_noise_stddev),
                p.y + get_gaussian(c_noise_mean, c_noise_stddev);
        mvSLAM::Point2Uncertainty covar(mvSLAM::Matrix2Type::Identity() * mvSLAM::sqr(c_noise_stddev));

        // isotropic covar
        p2_estimate.emplace_back(mean, covar);
    }

    /* Generate initial guess */
    // camera pose 
    mvSLAM::Vector6Type delta_pose;
    delta_pose << get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3), // translation
                  get_gaussian(0.0, 1e-2), get_gaussian(0.0, 1e-2), get_gaussian(0.0, 1e-2); // rotation
    mvSLAM::Pose pose2in1_scaled_guess = mvSLAM::SE3::exp(delta_pose) * P1 *  P2.inverse();

    // points
    std::vector<mvSLAM::Point3> pointsin1_scaled_guess;
    pointsin1_scaled_guess.reserve(points_in_world_frame.size());

    const mvSLAM::ScalarType p_noise_mean(0.0);
    const mvSLAM::ScalarType p_noise_stddev(5e-3);
    for (const auto &p : points_in_world_frame)
    {
        mvSLAM::Point3 mean;
        for (int i = 0; i < 3; ++i)
            mean[i] = p[i] + get_gaussian(p_noise_mean, p_noise_stddev);

        pointsin1_scaled_guess.emplace_back(mean);
    }

    
    // refinement 
    mvSLAM::PoseEstimate pose2in1_scaled_estimate;
    std::vector<mvSLAM::Point3Estimate> pointsin1_scaled_estimate;
    if (!refine_scene(p1_estimate,
                      p2_estimate,
                      K,
                      pose2in1_scaled_guess,
                      pointsin1_scaled_guess,
                      pose2in1_scaled_estimate,
                      pointsin1_scaled_estimate))
    {
        FAIL("refine_scene() failed");
    }

#ifdef DEBUG_OUTPUT
    // debug output
    {
        std::cout<<"====================================="<<std::endl;
        std::cout<<"Camera 2 pose in camera 1 ref frame:\n"
                 <<P2.inverse()<<std::endl;
        std::cout<<"[Refined] Camera 2 pose in camera 1 ref frame:\n"
                 <<pose2in1_scaled_estimate.mean()<<std::endl;
        std::cout<<"Points:"<<std::endl;
        for (const auto &p : points_in_world_frame)
        {
            std::cout<<p<<std::endl<<std::endl;
        }
        std::cout<<"[Refined] Points:"<<std::endl;
        for (const auto &pe : pointsin1_scaled_estimate)
        {
            std::cout<<pe.mean()<<std::endl<<std::endl;;
        }
    }
#endif 

    auto se3_2to1_recovered = pose2in1_scaled_estimate.mean().ln();
    for (size_t i = 0; i < 6; ++i)
        ASSERT_EQUAL(se3_2to1[i], se3_2to1_recovered[i], tolerance);
    ASSERT_TRUE(points_in_world_frame.size() == pointsin1_scaled_estimate.size());
    for (size_t i = 0; i < points_in_world_frame.size(); ++i)
    {
        const auto &pointin1_recovered = pointsin1_scaled_estimate[i].mean();
        for (size_t j = 0; j < 3; ++j)
            ASSERT_EQUAL(points_in_world_frame[i][j], pointin1_recovered[j], tolerance);
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
