#pragma once
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <base/space.hpp>
#include <vector>

namespace mvSLAM
{

pcl::PointCloud<pcl::PointXYZ>::Ptr
Vector3Type_to_PointCloud(std::vector<Vector3Type> &points);

pcl::PointCloud<pcl::PointXYZ>::Ptr
Point3_to_PointCloud(std::vector<Point3> &points);

/** Add a polyhedro representing the camera pose to the viewer.
 * @param [in] camera_pose  transformation from camera to world.
 */
void add_camera_representation(const Transformation &camera_pose,
                               const char *title,
                               pcl::visualization::PCLVisualizer &viewer,
                               bool shrink=false);

void initialize_visualizer(pcl::visualization::PCLVisualizer &viewer);

}
