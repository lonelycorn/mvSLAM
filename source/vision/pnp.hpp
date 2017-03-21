#pragma once
#include <base/image.hpp>
#include <base/space.hpp>

#include <vector>

namespace mvSLAM
{
/** Solve for camera pose in world ref frame from correspondences between
 * world points and image points.
 *
 * @param [in] world_points corresponding world points.
 * @param [in] image_points corresponding image points;
 *      already converted to ideal pinhold camera model.
 * @param [out] pose    pose of camera expressed in world frame.
 * @return true if a solution is found.
 */
bool pnp_solve(const std::vector<Point3> &world_points,
               const std::vector<IdealCameraImagePoint> &image_points,
               Transformation &pose);

/** Estimate camera pose in world ref frame from correspondences between
 * world points and image points.
 * A.k.a Motion-only Bundle Adjustment.
 *
 * @param [in] world_point_estimates    estimates of corresponding world points.
 * @param [in] image_point_estimates    estimates of image points.
 * @param [in] K    intrinsics of the camera.
 *      Use Identity if points are already converted to ideal pinhole camera.
 * @param [in] pose_guess   initial guess of camera pose in world ref frame.
 * @param [out] pose_estimate   refined estimate of camera pose in world ref frame.
 * @return true if the refinement was successful.
 */
bool pnp_refine(const std::vector<Point3Estimate> &world_point_estimates,
                const std::vector<Point2Estimate> &image_point_estimates,
                const CameraIntrinsics &K,
                const Transformation &pose_guess,
                TransformationEstimate &pose_estimate);
}
