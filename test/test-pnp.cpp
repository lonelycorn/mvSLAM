#include <base/image.hpp>
#include <vision/pnp.hpp>
#include <vision/camera.hpp>

#include "unit-test.hpp"
#include "unit-test-helper.hpp"

#include <iostream>

using namespace unit_test;

//#define DEBUG_OUTPUT

UNIT_TEST(pnp_solve_cube)
{
    constexpr mvSLAM::ScalarType tolerance = 0.001;
    /* Ground Truth */
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::Vector6Type se3; // from camera to world
    se3 << 1, 0, 0, // translation in positive x
           0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P(mvSLAM::SE3::exp(se3).inverse()); // from world to camera

    mvSLAM::PinholeCamera c(K, P);

    // generate points on a cube
    mvSLAM::SO3 rotation(0.0, 0.0, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 1.0;
    std::vector<mvSLAM::Point3> points =
        get_rig_points(RIG_TYPE::CUBE, rotation, translation, scale);

    // project image points
    std::vector<mvSLAM::ImagePoint> image_points = c.project_points(points);

    // PnP
    mvSLAM::Transformation pose_recovered;
    if (!pnp_solve(points,
                   image_points,
                   K,
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

UNIT_TEST(pnp_refine_L_shape)
{
    constexpr mvSLAM::ScalarType tolerance = 0.025;
    /* Ground Truth */
    mvSLAM::CameraIntrinsics K = mvSLAM::Matrix3Type::Identity();
    mvSLAM::Vector6Type se3; // from camera to world
    se3 << 1, 0, 0, // translation in positive x
           0, 0, 0; // no rotation
    mvSLAM::CameraExtrinsics P(mvSLAM::SE3::exp(se3).inverse()); // from world to camera

    mvSLAM::PinholeCamera c(K, P);


    // generate points on an L-shaped rig
    mvSLAM::SO3 rotation(1.5, 0.7, 0.0); // yaw, pitch, roll
    mvSLAM::Vector3Type translation{0.6, 0.0, 3.0};
    mvSLAM::ScalarType scale = 0.5;
    std::vector<mvSLAM::Point3> world_points =
        get_rig_points(RIG_TYPE::L_SHAPE, rotation, translation, scale);


    /* Simulate measurements */
    // projected image points
    std::vector<mvSLAM::ImagePoint> image_points = c.project_points(world_points);
    std::vector<mvSLAM::Point2Estimate> image_point_estimates;
    image_point_estimates.reserve(image_points.size());

    const mvSLAM::ScalarType c_noise_stddev(5e-3); // NOTE this is in pixels
    const mvSLAM::ScalarType c_noise_mean(0.0); // SHOULD be 0.

    for (const auto &p : image_points)
    {
        // additive noise
        mvSLAM::Point2 mean;
        mean << p.x + get_gaussian(c_noise_mean, c_noise_stddev),
                p.y + get_gaussian(c_noise_mean, c_noise_stddev);
        // isotropic covar
        mvSLAM::Point2Uncertainty covar(mvSLAM::Matrix2Type::Identity() * mvSLAM::sqr(c_noise_stddev));

        image_point_estimates.emplace_back(mean, covar);
    }

    /* World point prior */
    std::vector<mvSLAM::Point3Estimate> world_point_estimates;
    world_point_estimates.reserve(world_points.size());

    const mvSLAM::ScalarType p_noise_stddev(5e-3); // NOTE this is in meters
    const mvSLAM::ScalarType p_noise_mean(0.0); // SHOULD be 0.
    
    for (const auto &p : world_points)
    {
        // additive noise
        mvSLAM::Point3 mean;
        mean << p[0] + get_gaussian(p_noise_mean, p_noise_stddev),
                p[1] + get_gaussian(p_noise_mean, p_noise_stddev),
                p[2] + get_gaussian(p_noise_mean, p_noise_stddev);

        // isotropic covar
        mvSLAM::Point3Uncertainty covar(mvSLAM::Matrix3Type::Identity() * mvSLAM::sqr(p_noise_stddev));
        world_point_estimates.emplace_back(mean, covar);
    }

    /* Generate initial guess */
    mvSLAM::Vector6Type delta_pose;
    delta_pose << get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3), // translation
                  get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3), get_gaussian(0.0, 5e-3); // rotation
    mvSLAM::Transformation pose_guess = mvSLAM::SE3::exp(delta_pose) * P.inverse();

    // refinement 
    mvSLAM::TransformationEstimate pose_estimate;
    mvSLAM::ScalarType error;
    if (!pnp_refine(world_point_estimates,
                    image_point_estimates,
                    K,
                    pose_guess,
                    pose_estimate,
                    error))
    {
        FAIL("pnp_refine() failed");
    }

#ifdef DEBUG_OUTPUT
    // debug output
    {
        std::cout<<"====================================="<<std::endl;
        std::cout<<"Camera pose =\n"<<P.inverse()<<std::endl;
        std::cout<<"[Refined] Camera pose =\n"<<pose_estimate.mean()<<std::endl;
    }
#endif 

    auto se3_recovered = pose_estimate.mean().ln(); // camera to world
    std::cout<<"se3 = \n"<<se3<<std::endl;
    std::cout<<"se3_recovered = \n"<<se3_recovered<<std::endl;
    for (size_t i = 0; i < 6; ++i)
        ASSERT_EQUAL(se3[i], se3_recovered[i], tolerance);

    PASS();
}


int main()
{
    RUN_ALL_TESTS();
}
