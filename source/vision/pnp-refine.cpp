#include <base/math.hpp>
#include <cassert> 

#include <base/debug.hpp>
#include <base/gtsam.hpp>
#include <vision/pnp.hpp>

#include <gtsam/geometry/Point2.h> // image points
#include <gtsam/geometry/Point3.h> // world points
#include <gtsam/inference/Symbol.h> // use symbols like "X1"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h> // simple camera reprojection model
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

namespace mvSLAM
{
static Logger logger("pnp-refine", true);

static constexpr ScalarType
    PNP_REGULATOR_STDDEV_POSITION(1e-2);
static constexpr ScalarType
    PNP_REGULATOR_STDDEV_ORIENTATION(1e-2);

bool pnp_refine(const std::vector<Point3Estimate> &world_point_estimates,
                const std::vector<Point2Estimate> &image_point_estimates,
                const CameraIntrinsics &ci,
                const Transformation &pose_guess,
                TransformationEstimate &pose_estimate)
{
    assert(world_point_estimates.size() == image_point_estimates.size());
    logger.info("Refining PnP.");

    const size_t point_count = world_point_estimates.size();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_guess;

    // Camera Intrinsics: fx, fy, shear, px, py
    auto K = boost::make_shared<gtsam::Cal3_S2>(ci(0, 0), ci(1, 1), ci(0, 1),
                                                ci(0, 2), ci(1, 2));

    // add camera measurements
    for (size_t i = 0; i < point_count; ++i)
    {
        const Point2Estimate &p = image_point_estimates[i];
        auto observation = mvSLAM_Point2_to_gtsam_Point2(p.mean());
        auto uncertainty = gtsam::noiseModel::Gaussian::Covariance(p.covar());

        auto camera_symbol = gtsam::Symbol('x', 1);
        auto world_point_symbol = gtsam::Symbol('p', i);
        auto f = 
            gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
            (observation, uncertainty, camera_symbol, world_point_symbol, K);
        graph.push_back(f);
    }
    
    // add point estimates
    for (size_t i = 0; i < point_count; ++i)
    {
        const Point3Estimate &p = world_point_estimates[i];
        const auto symbol = gtsam::Symbol('p', i);
        auto position = mvSLAM_Point3_to_gtsam_Point3(p.mean());
        auto noise = gtsam::noiseModel::Gaussian::Covariance(p.covar());
        graph.push_back(gtsam::PriorFactor<gtsam::Point3>(symbol, position, noise));
        initial_guess.insert(symbol, position);
    }

    // add camera
    {
        const auto symbol = gtsam::Symbol('x', 1);
        auto pose = SE3_to_Pose3(pose_guess);
        // NOTE: it is always good to use a regulator to make sure that the final estimate
        // does not deviate too much from the initial guess.
        gtsam::Vector6 stddev;
        const ScalarType prior_stddev_position = PNP_REGULATOR_STDDEV_POSITION;
        const ScalarType prior_stddev_orientation = PNP_REGULATOR_STDDEV_ORIENTATION;
        // NOTE: translation part first
        stddev << prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_orientation,
                  prior_stddev_orientation,
                  prior_stddev_orientation;
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(stddev);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(symbol, pose, noise));
        initial_guess.insert(symbol, pose);
    }
#ifdef DEBUG_PNP
    graph.print("\n\n===== Factor Graph =====\n");
    initial_guess.print("\n\n===== Initial Guess =====\n");
#endif

    // solve
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess);
    gtsam::Values result = optimizer.optimize();
#ifdef DEBUG_PNP
    result.print("\n\n===== Result =====");
#endif
    
    // convert to output types
    gtsam::Marginals marginals(graph, result); // to calculate the covariances
    {
        auto symbol = gtsam::Symbol('x', 1);
        gtsam::Pose3 pose_gtsam = result.at<gtsam::Pose3>(symbol); // in world frame
        pose_estimate.mean() = Pose3_to_SE3(pose_gtsam);
        pose_estimate.covar() = marginals.marginalCovariance(symbol);
    }
    // FIXME: we are __NOT__ updating the point estimates, despite that there is
    // new information from the new iamge.
    return true;
}

}
