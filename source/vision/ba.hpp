#pragma once
#include <base/space.hpp>
#include <vision/camera.hpp>
#include <unordered_set>
#include <unordered_map>

namespace mvSLAM
{

using PointIdToPoint2Estimate = std::unordered_map<Id::Type, Point2Estimate>;

/** bundle adjustment on both frame poses and points.
 * @param [in] ci   intrisics of the camera
 * @param [in] frame_id a set of unique identifier of all related frames.
 * @param [in] point_id a set of unique identifier of all related points.
 * @param [in] frame_pose_guess initial guess of frame poses, indexed by frame id.
 * @param [in] frame_pose_prior prior (if any) of frame poses, indexed by frame id.
 * @param [in] point_guess  initial guess of points, indexed by point id.
 * @param [in] point_prior  prior (if any) of points, indexed by point id.
 * @param [in] frame_observation    first indexed by frame id, then by point id.
 * @param [out] frame_pose_estimate refined frame pose estimates, indexed by frame id.
 * @param [out] point_estimate  refine point estimates, indexed by point id.
 * @param [out] final_error error after refinement.
 */
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
        ScalarType &final_error);


}

