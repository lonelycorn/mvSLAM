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
Point3D_to_PointCloud(std::vector<Point3D> &points);

/**
 * @brief Add a polyhedro representing the camera pose to the viewer.
 */
void add_camera_representation(const Pose &camera_pose,
                               const char *title,
                               pcl::visualization::PCLVisualizer &viewer);

void initialize_visualizer(pcl::visualization::PCLVisualizer &viewer);

}
