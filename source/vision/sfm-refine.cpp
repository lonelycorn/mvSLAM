#include <cassert>

#include <base/debug.hpp>
#include <vision/ba.hpp>
#include <vision/sfm.hpp>

namespace mvSLAM
{
static Logger logger("[sfm-refine]", false);

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
                std::vector<Point3Estimate> &pointsin1_estimate,
                ScalarType &error)
{
    assert(p1_estimate.size() == p2_estimate.size());
    assert(p1_estimate.size() == pointsin1_guess.size());
    const size_t point_count = p1_estimate.size();
    logger.info("point count = ", point_count);

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
    frame_id.insert(1);

    // point id
    for (size_t i = 0; i < point_count; ++i)
    {
        point_id.insert(i);
    }

    // frame pose guess
    frame_pose_guess[0] = SE3(); // camera 1 is at the origin
    frame_pose_guess[1] = pose2in1_guess; // camera 2 in camera 1 ref frame

    // frame pose prior
    {
        // fix world origin
        Matrix6Type covar(Matrix6Type::Identity());
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= sqr(SFM_ANCHOR_STDDEV_POSITION);
            covar(i+3, i+3) *= sqr(SFM_ANCHOR_STDDEV_ORIENTATION);
        }
        frame_pose_prior[0] = covar;
    }
    {
        // regulator
        Matrix6Type covar(Matrix6Type::Identity());
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= sqr(SFM_REGULATOR_STDDEV_POSITION);
            covar(i+3, i+3) *= sqr(SFM_REGULATOR_STDDEV_ORIENTATION);
        }
        frame_pose_prior[1] = covar;
    }

    // point guess
    for (size_t i = 0; i < point_count; ++i)
    {
        point_guess[i] = pointsin1_guess[i];
    }

    // point prior
    for (size_t i = 0; i < point_count; ++i)
    {
        // regulator
        Matrix3Type covar(Matrix3Type::Identity());
        for (int j = 0; j < 3; ++j)
        {
            covar(j, j) *= sqr(SFM_REGULATOR_STDDEV_POSITION);
        }
        point_prior[i] = covar;
    }

    // frame observation
    for (int camera_index = 0; camera_index < 2; ++camera_index)
    {
        const auto &p_estimate = (camera_index == 0 ? p1_estimate : p2_estimate);

        PointIdToPoint2Estimate observation;
        for (size_t i = 0; i < point_count; ++i)
        {
            observation[i] = p_estimate[i];
        }
        frame_observation[camera_index] = observation;
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

    // conver to output types
    // FIXME: not sure if the covar of camera 2 estimate is correct. the covar has
    //        3 components: camera 1 prior (to fix the world origin), camera 2 and
    //        points' prior (to not drift away), and the real uncertainty from image
    //        point observations. I think we should remove the first 2 contributions.
    pose2in1_estimate = frame_pose_estimate[1];
    pointsin1_estimate.clear();
    pointsin1_estimate.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        pointsin1_estimate.push_back(point_estimate[i]);
    }

    return true;
}

}
