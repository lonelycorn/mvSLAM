#include <cassert>
#include <iostream>

#include <base/debug.hpp>
#include <base/gtsam.hpp>
#include <vision/sfm.hpp>

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
static Logger logger("sfm-refine", true);

static constexpr ScalarType
    SFM_ANCHOR_STDDEV_POSITION(1e-5);
static constexpr ScalarType
    SFM_ANCHOR_STDDEV_ORIENTATION(1e-5);
static constexpr ScalarType
    SFM_REGULATOR_STDDEV_POSITION(1e-2);
static constexpr ScalarType
    SFM_REGULATOR_STDDEV_ORIENTATION(1e-2);

bool sfm_refine(const std::vector<Point2Estimate> &p1_estimate,
                const std::vector<Point2Estimate> &p2_estimate,
                const CameraIntrinsics &ci,
                const Transformation &pose2in1_guess,
                const std::vector<Point3> pointsin1_guess,
                TransformationEstimate &pose2in1_estimate,
                std::vector<Point3Estimate> &pointsin1_estimate)
{
    assert(p1_estimate.size() == p2_estimate.size());
    assert(p1_estimate.size() == pointsin1_guess.size());
    logger.info("Refining SfM.");

    const size_t point_count = p1_estimate.size();

    gtsam::NonlinearFactorGraph graph;

    // Camera Intrinsics: fx, fy, shear, px, py
    auto K = boost::make_shared<gtsam::Cal3_S2>(ci(0, 0), ci(1, 1), ci(0, 1),
                                                ci(0, 2), ci(1, 2));

    // Add camera measurements
    for (int camera_index = 1; camera_index <= 2; ++camera_index)
    {
        const auto &p_estimate = (camera_index < 2) ? p1_estimate : p2_estimate;
        for (size_t i = 0; i < point_count; ++i)
        {
            const Point2Estimate &p = p_estimate[i];
            auto observation = mvSLAM_Point2_to_gtsam_Point2(p.mean());
            auto uncertainty = gtsam::noiseModel::Gaussian::Covariance(p.covar());

            auto camera1_symbol = gtsam::Symbol('x', camera_index);
            auto point_symbol = gtsam::Symbol('p', i);
            auto f =
                gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                (observation, uncertainty, camera1_symbol, point_symbol, K);
            graph.push_back(f);
        }
    }

    // Use gtsam::PriorFactor to regulate camera 1 pose.
    // Note that the optimization function is invariant to ANY rigid transformation.
    // To uniquely determine the solution, the covar MUST be small to fix the reference frame. 
    {
        auto symbol = gtsam::Symbol('x', 1);
        auto pose = gtsam::Pose3(); // origin
        gtsam::Vector6 stddev;
        const ScalarType prior_stddev_position = SFM_ANCHOR_STDDEV_POSITION;
        const ScalarType prior_stddev_orientation = SFM_ANCHOR_STDDEV_ORIENTATION;
        // NOTE: translation before rotation
        stddev << prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_orientation,
                  prior_stddev_orientation,
                  prior_stddev_orientation;
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(stddev);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(symbol, pose, noise));
    }

    // Use gtsam::PriorFactor to regulate camera 2 pose.
    {
        auto symbol = gtsam::Symbol('x', 2);
        auto pose = SE3_to_Pose3(pose2in1_guess);
        gtsam::Vector6 stddev;
        const ScalarType prior_stddev_position = SFM_REGULATOR_STDDEV_POSITION;
        const ScalarType prior_stddev_orientation = SFM_REGULATOR_STDDEV_ORIENTATION;
        // NOTE: translation before rotation
        stddev << prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_position,
                  prior_stddev_orientation,
                  prior_stddev_orientation,
                  prior_stddev_orientation;
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(stddev);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(symbol, pose, noise));
    }
    // Use gtsam::PriorFactor to regulate all point positions
    {
        for (size_t i = 0; i < point_count; ++i)
        {
            auto symbol = gtsam::Symbol('p', i);
            auto p = mvSLAM_Point3_to_gtsam_Point3(pointsin1_guess[i]);
            gtsam::Vector3 stddev;
            const ScalarType prior_stddev_position = SFM_REGULATOR_STDDEV_POSITION;
            stddev << prior_stddev_position,
                      prior_stddev_position,
                      prior_stddev_position;
            auto noise = gtsam::noiseModel::Diagonal::Sigmas(stddev);
            graph.push_back(gtsam::PriorFactor<gtsam::Point3>(symbol, p, noise));
        }
    }
#ifdef DEBUG_SFM
    graph.print("\n\n===== Factor Graph =====\n");
#endif

    // initial values
    gtsam::Values initial_guess;
    {
        auto symbol = gtsam::Symbol('x', 1);
        auto pose = gtsam::Pose3();
        initial_guess.insert(symbol, pose);
    }
    {
        auto symbol = gtsam::Symbol('x', 2);
        auto pose = SE3_to_Pose3(pose2in1_guess);
        initial_guess.insert(symbol, pose);
    }
    for (size_t i = 0; i < point_count; ++i)
    {
        auto symbol = gtsam::Symbol('p', i);
        auto p = mvSLAM_Point3_to_gtsam_Point3(pointsin1_guess[i]);
        initial_guess.insert(symbol, p);
    }
#ifdef DEBUG_SFM
    initial_guess.print("\n\n===== Initial Guess =====\n");
#endif

    // solve
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess);
    gtsam::Values result = optimizer.optimize();
#ifdef DEBUG_SFM
    result.print("\n\n===== Result =====\n");
#endif

    // convert to output type
    gtsam::Marginals marginals(graph, result); // to calculate the covariances
    {
        auto symbol_x1 = gtsam::Symbol('x', 1);
        auto symbol_x2 = gtsam::Symbol('x', 2);
        gtsam::Pose3 pose1_gtsam = result.at<gtsam::Pose3>(symbol_x1); // in world frame
        gtsam::Pose3 pose2_gtsam = result.at<gtsam::Pose3>(symbol_x2); // in world frame
        gtsam::Pose3 pose2in1_gtsam = pose1_gtsam.inverse() * pose2_gtsam;
        pose2in1_estimate.mean() = Pose3_to_SE3(pose2in1_gtsam);
        // FIXME: we treat x1 as fixed... but is that correct? we should pass in and update x1 estimate
        pose2in1_estimate.covar() = marginals.marginalCovariance(symbol_x2);
    }
    pointsin1_estimate.clear();
    pointsin1_estimate.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        auto symbol = gtsam::Symbol('p', i);
        gtsam::Point3 point_gtsam = result.at<gtsam::Point3>(symbol); // in world frame
        auto mean = gtsam_Point3_to_mvSLAM_Point3(point_gtsam);
        auto covar = marginals.marginalCovariance(symbol);
        pointsin1_estimate.emplace_back(mean, covar);
    }

    return true;
}

}
