#pragma once
#include <base/image.hpp>
#include <base/data-type.hpp>
#include <vision/camera.hpp>
#include <vision/visual-feature.hpp>
#include <vector>

namespace mvSLAM
{

/** Solve for relative motion and structure between a pair of iamges.
 * Internally, the reconstruction problem is solved with Direct Linear
 * Transform method. 
 * Due to the nature of SfM, the reconstructed world could only be
 * determined up to scale.
 *
 * @param [in] p1   matching points from camera 1 image.
 * @param [in] p2   matching points from camera 2 image.
 * @param [in] K    Camera intrinsics.
 *      Use Identity if @p p1 and @p p2 are from an ideal pinhole camera.
 * @param [out] pose2in1_scaled     pose of camera 2 expressed in
 *      camera 1 ref frame. This is also the transform from camera
 *      2 ref frame to camera 1 ref frame. Due to the scale ambiguity,
 *      the translation part is a unit vector.
 * @param [out] pointsin1_scaled    position of world points.
 *      The coordinates are scaled by the same factor as @p points2in1_scaled.
 * @param [out] points_index the original indices of the reconstructed points.
 * @return true if the reconstruction was successful.
 */
bool sfm_solve(const std::vector<ImagePoint> &p1,
               const std::vector<ImagePoint> &p2,
               const CameraIntrinsics &K,
               Transformation &pose2in1_scaled,
               std::vector<Point3> &pointsin1_scaled,
               std::vector<size_t> &point_indexes);

/** Triangulate points
 * @param [in] p1   matching points from camera 1 image.
 * @param [in] p2   matching points from camera 2 image.
 * @param [in] K    Camera intrinsics.
 *      Use Identity if @p p1 and @p p2 are from an ideal pinhole camera.
 * @param [in] pose1    camera 1 pose expressed in world ref frame.
 * @param [in] pose2    camera 2 pose expressed in world ref frame.
 * @param [out] points  triangulated point locations in world.
 * @param [out] point_indexs    original indexes of @p points.
 */
void sfm_triangulate(const std::vector<ImagePoint> &p1,
                     const std::vector<ImagePoint> &p2,
                     const CameraIntrinsics &K,
                     const Transformation &pose1,
                     const Transformation &pose2,
                     std::vector<Point3> &points,
                     std::vector<size_t> &point_indexes);


/** Refine relative motion and structure using initial guesses.
 *
 * @param [in] p1_estimates estimates of matching points from camera 1 image;
 * @param [in] p2_estimates estimates of matching points from camera 2 image;
 * @param [in] K    camera intrinsics.
 *      use Identity if points are already converted to ideal cameras.
 * @param [in] pose2in1_guess   initial guess for camera 2 pose in camera 1 ref frame.
 * @param [in] pointsin1_guess  initial guesses for world points in camera 1 ref frame.
 * @param [out] pose2in1_estimate   refined estimate of camera 2 pose in camera 1 ref frame.
 * @param [out] pointsin1_guess refined estimates of world points in camera 1 ref frame.
 * @param [out] error   the error after the refinement.
 * @return true if the refinement was successful.
 */
bool sfm_refine(const std::vector<Point2Estimate> &p1_estimates,
                const std::vector<Point2Estimate> &p2_estimates,
                const CameraIntrinsics &K,
                const Transformation &pose2in1_guess,
                const std::vector<Point3> pointsin1_guess,
                TransformationEstimate &pose2in1_estimate,
                std::vector<Point3Estimate> &pointsin1_estimate,
                ScalarType &error);
}

