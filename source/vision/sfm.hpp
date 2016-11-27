#pragma once
#include <base/image.hpp>
#include <base/space.hpp>
#include <vision/visual-feature.hpp>
#include <string>

namespace mvSLAM
{

/**
 * @brief
 * @param [in] image1   image taken at camera pose 1.
 * @prarm [in] image2   image taken at camera pose 2.
 * @param [in] K    intrinsics of the camera.
 * @param [out] pose2in1_scaled camera pose 2 in camera 1 ref frame.
 *      the translation is scaled to 1.
 * @param [out] pointsin1_scaled    reconstructed points in camera 1 ref frame.
 *      scaled by the same factor as @p pose2in1_scaled
 * @return whether the reconstruction was successful.
*/
/*
bool reconstruct_scene(const ImageGrayscale &image1,
                       const ImageGrayscale &image2,
                       const CameraIntrinsics &K,
                       Pose &pose2in1_scaled,
                       std::vector<Point3D> &pointsin1_scaled);
*/

// pose2in1: the pose of camera 2, expressed in camera 1 ref frame. This is the
// transform from camera 2 ref frame to camera 1 ref frame, and is the inverse
// of the camera 2 matrix.
bool reconstruct_scene(const std::vector<NormalizedPoint> &normalized_points1,
                       const std::vector<NormalizedPoint> &normalized_points2,
                       Pose &pose2in1_scaled,
                       std::vector<Point3D> &pointsin1_scaled);
                       

/**
 * vf1 and vf2 contain matched visual features
 */
bool refine_scene(const VisualFeature &vf1,
                  const VisualFeature &vf2,
                  const CameraIntrinsics &K,
                  const Pose &point2in1_scaled_guess,
                  const std::vector<Point3D> pointsin1_scaled_guess,
                  PoseEstimate &pose_estimate_scaled,
                  std::vector<Point3DEstimate> &point_estimates_scaled);

}

