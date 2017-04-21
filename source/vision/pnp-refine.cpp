#include <cassert> 

#include <base/debug.hpp>
#include <base/space.hpp>
#include <vision/ba.hpp>
#include <vision/pnp.hpp>

namespace mvSLAM
{
static Logger logger("[pnp-refine]", false);

static constexpr ScalarType
    PNP_REGULATOR_STDDEV_POSITION(1e-2);
static constexpr ScalarType
    PNP_REGULATOR_STDDEV_ORIENTATION(1e-2);

bool pnp_refine(const std::vector<Point3Estimate> &world_point_estimates,
                const std::vector<Point2Estimate> &image_point_estimates,
                const CameraIntrinsics &ci,
                const Transformation &pose_guess,
                TransformationEstimate &pose_estimate,
                ScalarType &error)
{
    assert(world_point_estimates.size() == image_point_estimates.size());
    logger.info("Refining PnP.");

    const size_t point_count = world_point_estimates.size();

    std::unordered_set<Id::Type> frame_id;
    std::unordered_set<Id::Type> point_id;
    std::unordered_map<Id::Type, Transformation> frame_pose_guess;
    std::unordered_map<Id::Type, TransformationUncertainty> frame_pose_prior;
    std::unordered_map<Id::Type, Point3> point_guess;
    std::unordered_map<Id::Type, Point3Uncertainty> point_prior;
    std::unordered_map<Id::Type, PointIdToPoint2Estimate> frame_observation;
    std::unordered_map<Id::Type, TransformationEstimate> frame_pose_estimate;
    std::unordered_map<Id::Type, Point3Estimate> point_estimate;

    // frame id
    frame_id.insert(0);

    // point id
    for (size_t i = 0; i < point_count; ++i)
    {
        point_id.insert(i);
    }

    // frame_pose_guess
    frame_pose_guess[0] = pose_guess;

    // frame_pose_prior
    {
        // regulator
        Matrix6Type covar(Matrix6Type::Identity());
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= sqr(PNP_REGULATOR_STDDEV_POSITION);
            covar(i+3, i+3) *= sqr(PNP_REGULATOR_STDDEV_ORIENTATION);
        }
        frame_pose_prior[0] = covar;
    }

    // point guess
    for (size_t i = 0; i < point_count; ++i)
    {
        point_guess[i] = world_point_estimates[i].mean();
    }

    // point prior
    for (size_t i = 0; i < point_count; ++i)
    {
        point_prior[i] = world_point_estimates[i].covar();
    }

    // frame observation
    {
        PointIdToPoint2Estimate observation;
        for (size_t i = 0; i < point_count; ++i)
        {
            observation[i] = image_point_estimates[i];
        }
        frame_observation[0] = observation;
    }

    // do bundle adjustment
    ba_frame_pose_and_point(ci,
                            frame_id,
                            point_id,
                            frame_pose_guess,
                            frame_pose_prior,
                            point_guess,
                            point_prior,
                            frame_observation,
                            frame_pose_estimate,
                            point_estimate,
                            error);

    // convert to output types
    // FIXME: not sure if the covar of camera pose estimate is correct. the covar
    //        has 3 components: world point prior, camera pose prior (regulator),
    //        and the uncertainty from the image point observations. I think the
    //        we should remove the regulator contribution.
    pose_estimate = frame_pose_estimate[0];

    // FIXME: we are __NOT__ updating the point estimates, despite that there is
    // new information from the new iamge.

    return true;
}

}
