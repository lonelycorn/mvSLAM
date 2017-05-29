#pragma once
#include <base/image.hpp>
#include <math/space.hpp>
#include <vision/camera.hpp>

#include <vector>

namespace mvSLAM
{
/** Solve for camera pose in world ref frame from correspondences between
 * world points and image points.
 *
 * @param [in] world_points corresponding world points.
 * @param [in] image_points corresponding image points;
 * @param [in] K    intrinsics of the camera.
 *      Use Identity if @p image_points are from an ideal pinhole camera.
 * @param [out] pose    pose of camera expressed in world frame.
 * @param [out] inlier_point_indexes    inlier points' original indexes in
 *      the input arrays.
 * @return true if a solution is found.
 */
bool pnp_solve(const std::vector<Point3> &world_points,
               const std::vector<ImagePoint> &image_points,
               const CameraIntrinsics &K,
               Transformation &pose,
               std::vector<size_t> &inlier_point_indexes);

/** Estimate camera pose in world ref frame from correspondences between
 * world points and image points.
 * A.k.a Motion-only Bundle Adjustment.
 *
 * @param [in] world_point_estimates    estimates of corresponding world points.
 * @param [in] image_point_estimates    estimates of image points.
 * @param [in] K    intrinsics of the camera.
 *      Use Identity if @p image_point_estimates are from an ideal pinhole camera.
 * @param [in] pose_guess   initial guess of camera pose in world ref frame.
 * @param [out] pose_estimate   refined estimate of camera pose in world ref frame.
 * @param [out] error   error after refinement.
 * @return true if the refinement was successful.
 */
bool pnp_refine(const std::vector<Point3Estimate> &world_point_estimates,
                const std::vector<Point2Estimate> &image_point_estimates,
                const CameraIntrinsics &K,
                const Transformation &pose_guess,
                TransformationEstimate &pose_estimate,
                ScalarType &error);
}
