#pragma once
#include <base/image.hpp>
#include <base/space.hpp>
#include <vision/visual-feature.hpp>
#include <string>

namespace mvSLAM
{

/**
 * @brief Estimate the camera pose of image 2 relative to that
          of image 1.
 *        This is done by dissecting the Essential matrix.
 * @param [in] image1   image taken at camera pose 1.
 * @prarm [in] image2   image taken at camera pose 2.
 * @param [in] intrinsics   intrinsics of the camera
 * @param [out] pose2in1    camera pose 2 in camera pose 1 ref frame.
 * @return whether the estimation was successful.
 */
/*
bool estimate_relative_pose(const VisualFeature &vf1,
                            const VisualFeature &vf2,
                            const CameraIntrinsics &K,
                            Pose &pose2in1);
*/

/**
 * @brief
 * @param [in] vf1      visual feature extracted from image 1.
 * @param [in] vf2      visual feature extracted from image 2.
 * @prarm [in] pose2in1 camera pose 2 in camera pose 1 ref frame.
 * @param [in] intrinsics intrinsics of the camera.
 * @param [out] points  triangulated points, in camera pose 1 ref frame.
 * @return whether the triangulation was successful.
 */
/*
bool triangulate_points(const VisualFeature &vf1,
                        const VisualFeature &vf2,
                        const CameraIntrinsics &K,
                        const Pose &pose2in1,
                        std::vector<Point3D> &pointsin1);
*/

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
bool reconstruct_scene(const ImageGrayscale &image1,
                       const ImageGrayscale &image2,
                       const CameraIntrinsics &K,
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

