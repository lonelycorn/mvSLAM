#pragma once
#include <base/image.hpp>
#include <base/space.hpp>
#include <vision/visual-feature.hpp>
#include <string>
#include <vector>

namespace mvSLAM
{

/** Reconstruct relative motion and structure with linear methods.
 * @param [in] p1   matching points from camera 1 image;
 *      already converted to ideal camera model.
 * @param [in] p2   matching points from camera 2 image;
 *      already converted to ideal camera model.
 * @param [out] pose2in1_scaled     pose of camera 2 expressed in
 *      camera 1 ref frame. This is also the transform from camera
 *      2 ref frame to camera 1 ref frame. Due to the scale ambiguity,
 *      the translation part is a unit vector.
 * @param [out] pointsin1_scaled    position of world points.
 *      The coordinates are scaled by the same factor as @p points2in1_scaled.
 * @return true if the reconstruction was successful.
 */
bool reconstruct_scene(const std::vector<IdealCameraImagePoint> &p1,
                       const std::vector<IdealCameraImagePoint> &p2,
                       Pose &pose2in1_scaled,
                       std::vector<Point3> &pointsin1_scaled);
                       

/** Refine relative motion and structure using initial guesses.
 * @param [in] p1_estimates estimate of matching points from camera 1 image;
 * @param [in] p2_estimates estimate of matching points from camera 2 image;
 * @param [in] K    camera intrinsics.
 *      use Identity if points are already converted to ideal cameras.
 * @param [in] pose2in1_scaled_guess    initial guess for camera 2 pose in
 *      camera 1 ref frame, the translation of which is scaled to a unit vector.
 * @param [in] pointsin1_scaled_guess   initial guess for point locations in
 *      camera 1 ref frame, scaled by the same factor as @p pose2in1_scaled_guess.
 * @param [out] pose2in1_scaled_estimate    refined estimate of camera 2 pose in
 *      camera 1 ref frame, scaled by the same factor as @p pose2in1_scaled_guess.
 * @param [out] pointsin1_scaled_guess  refined estimate of point locations in
 *      camera 1 ref frame, scaled by the same factor as @p pose2in1_scaled_guess.
 * @return true if the refinement was successful.
 */
bool refine_scene(const std::vector<Point2Estimate> &p1_estimates,
                  const std::vector<Point2Estimate> &p2_estimates,
                  const CameraIntrinsics &K,
                  const Pose &pose2in1_scaled_guess,
                  const std::vector<Point3> pointsin1_scaled_guess,
                  PoseEstimate &pose2in1_scaled_estimate,
                  std::vector<Point3Estimate> &pointsin1_scaled_estimate);

}

