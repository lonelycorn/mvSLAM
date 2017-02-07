#pragma once
#include <base/image.hpp>
#include <base/space.hpp>
#include <vision/visual-feature.hpp>
#include <string>

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
                       

/**
 * vf1 and vf2 contain matched visual features
 */
bool refine_scene(const VisualFeature &vf1,
                  const VisualFeature &vf2,
                  const CameraIntrinsics &K,
                  const Pose &point2in1_scaled_guess,
                  const std::vector<Point3> pointsin1_scaled_guess,
                  PoseEstimate &pose_estimate_scaled,
                  std::vector<Point3Estimate> &point_estimates_scaled);

}

