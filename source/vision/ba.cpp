#include <cassert>

#include <vision/ba.hpp>
#include <base/gtsam.hpp>
#include <base/debug.hpp>

#include <gtsam/geometry/Point2.h> // image points
#include <gtsam/geometry/Point3.h> // world points
#include <gtsam/inference/Symbol.h> // use symbols like "X1"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h> // simple camera reprojection model
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

//#define DEBUG_BA

namespace mvSLAM
{
static Logger logger("[ba]", false);

static constexpr char SymbolPrefixFrame = 'x';
static constexpr char SymbolPrefixPoint = 'p';

void ba_frame_pose_and_point(
        const CameraIntrinsics &ci,
        const std::unordered_set<Id::Type> &frame_id,
        const std::unordered_set<Id::Type> &point_id,
        const std::unordered_map<Id::Type, Transformation> &frame_pose_guess,
        const std::unordered_map<Id::Type, TransformationUncertainty> &frame_pose_prior,
        const std::unordered_map<Id::Type, Point3> &point_guess,
        const std::unordered_map<Id::Type, Point3Uncertainty> &point_prior,
        const std::unordered_map<Id::Type, PointIdToPoint2Estimate> &frame_observation,
        std::unordered_map<Id::Type, TransformationEstimate> &frame_pose_estimate,
        std::unordered_map<Id::Type, Point3Estimate> &point_estimate,
        ScalarType &final_error)
{
    assert(frame_id.size() > 0);
    assert(point_id.size() > 0);
    assert(frame_pose_guess.size() == frame_id.size());
    assert(point_guess.size() == point_id.size());
    assert(frame_pose_prior.size() + point_prior.size() >= 2); // need at least 2 prior to anchor the ref frame and the scale.
    assert(frame_observation.size() > 0);

    // Camera Intrinsics: fx, fy, shear, px, py
    auto K = boost::make_shared<gtsam::Cal3_S2>(ci(0, 0), ci(1, 1), ci(0, 1),
                                                ci(0, 2), ci(1, 2));

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_guess;

    // frame pose initial guess and prior
    for (auto &fid : frame_id)
    {
        assert(frame_pose_guess.count(fid) == 1);
        assert(frame_pose_prior.count(fid) <= 1);

        auto symbol = gtsam::Symbol(SymbolPrefixFrame, fid);
        auto value = SE3_to_Pose3(frame_pose_guess.at(fid));

        initial_guess.insert(symbol, value);

        // add prior
        if (frame_pose_prior.count(fid) == 1)
        {
            const auto &covar = frame_pose_prior.at(fid);
            auto noise = gtsam::noiseModel::Gaussian::Covariance(covar);
            gtsam::PriorFactor<gtsam::Pose3> factor(symbol, value, noise);
            graph.push_back(factor);
        }
    }

    // point initial guess and prior
    for (auto &pid : point_id)
    {
        assert(point_guess.count(pid) == 1);
        assert(point_prior.count(pid) <= 1);

        auto symbol = gtsam::Symbol(SymbolPrefixPoint, pid);
        auto value = mvSLAM_Point3_to_gtsam_Point3(point_guess.at(pid));

        initial_guess.insert(symbol, value);

        // add prior
        if (point_prior.count(pid) == 1)
        {
            const auto &covar = point_prior.at(pid);
            auto noise = gtsam::noiseModel::Gaussian::Covariance(covar);
            gtsam::PriorFactor<gtsam::Point3> factor(symbol, value, noise);
            graph.push_back(factor);
        }

    }

    // add frame observations
    for (const auto &fid_observation_pair : frame_observation)
    {
        auto fid = fid_observation_pair.first;
        assert(frame_id.count(fid) == 1);
        auto frame_symbol = gtsam::Symbol(SymbolPrefixFrame, fid);
        const auto &observation = fid_observation_pair.second;

        for (const auto &pid_estimate_pair : observation)
        {
            auto pid = pid_estimate_pair.first;
            assert(point_id.count(pid) == 1);
            auto point_symbol = gtsam::Symbol(SymbolPrefixPoint, pid);
            const auto &pe = pid_estimate_pair.second;
            auto value = mvSLAM_Point2_to_gtsam_Point2(pe.mean());
            auto noise = gtsam::noiseModel::Gaussian::Covariance(pe.covar());

            auto f = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
                    value, noise, frame_symbol, point_symbol, K);
            graph.push_back(f);
        }
    }

#ifdef DEBUG_BA
    graph.print("\n\n===== Factor Graph =====\n");
    initial_guess.print("\n\n===== Initial Guess =====\n");
#endif

    // solve
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess);
    gtsam::Values result = optimizer.optimize();
    gtsam::Marginals marginal(graph, result); // covar of linearized factor graph

#ifdef DEBUG_BA
    result.print("\n\n===== Result =====\n");
#endif

    // convert to output type
    frame_pose_estimate.clear();
    for (auto fid : frame_id)
    {
        auto symbol = gtsam::Symbol(SymbolPrefixFrame, fid);
        auto value = result.at<gtsam::Pose3>(symbol);

        auto mean = Pose3_to_SE3(value);
        auto covar = marginal.marginalCovariance(symbol);
        frame_pose_estimate[fid] = TransformationEstimate(mean, covar);
    }

    point_estimate.clear();
    for (auto pid : point_id)
    {
        auto symbol = gtsam::Symbol(SymbolPrefixPoint, pid);
        auto value = result.at<gtsam::Point3>(symbol);
        
        auto mean = gtsam_Point3_to_mvSLAM_Point3(value);
        auto covar = marginal.marginalCovariance(symbol);
        point_estimate[pid] = Point3Estimate(mean, covar);
    }
    final_error = optimizer.error();
}

}

