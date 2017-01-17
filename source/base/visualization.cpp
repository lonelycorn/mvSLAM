#include <system-config.hpp>
#include <base/visualization.hpp>
#include <string>
namespace mvSLAM
{

static constexpr ScalarType CAMERA_SIZE = (ScalarType) 1.0;

static std::vector<Vector3Type>
transform_points(const std::vector<Vector3Type> &points,
                 const SE3 &T)
{
    std::vector<Vector3Type> result; 
    for (const auto &p : points)
    {
        result.push_back(T * p);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Vector3Type_to_PointCloud(std::vector<Vector3Type> &points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : points)
    {
        result->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    
    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
Point3D_to_PointCloud(std::vector<Point3D> &points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : points)
    {
        result->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    
    return result;
}

void add_camera_representation(const Pose &camera_pose,
                               const char *title,
                               pcl::visualization::PCLVisualizer &viewer,
                               bool shrink)
{
    std::vector<Vector3Type> points;
    std::string prefix(title);

    ScalarType camera_size = CAMERA_SIZE;
    if (shrink)
        camera_size *= (ScalarType) 0.5;

    Vector3Type center; center << 0.0, 0.0, 0.0;
    Vector3Type maxmax; maxmax << 1.0, 1.0, 1.0; maxmax *= camera_size;
    Vector3Type maxmin; maxmin << 1.0,-1.0, 1.0; maxmin *= camera_size;
    Vector3Type minmin; minmin <<-1.0,-1.0, 1.0; minmin *= camera_size;
    Vector3Type minmax; minmax <<-1.0, 1.0, 1.0; minmax *= camera_size;

    {
        points.clear();
        points.push_back(maxmax);
        points.push_back(maxmin);
        points.push_back(minmin);
        points.push_back(minmax);
        auto transformed = transform_points(points, camera_pose);
        viewer.addPolygon<pcl::PointXYZ>(Vector3Type_to_PointCloud(transformed), prefix + "frontal");
    }

    {
        points.clear();
        points.push_back(maxmax);
        points.push_back(maxmin);
        points.push_back(center);
        auto transformed = transform_points(points, camera_pose);
        viewer.addPolygon<pcl::PointXYZ>(Vector3Type_to_PointCloud(transformed), prefix + "positve_x");
    }

    {
        points.clear();
        points.push_back(maxmin);
        points.push_back(minmin);
        points.push_back(center);
        auto transformed = transform_points(points, camera_pose);
        viewer.addPolygon<pcl::PointXYZ>(Vector3Type_to_PointCloud(transformed), prefix + "negative_y");
    }

    {
        points.clear();
        points.push_back(minmin);
        points.push_back(minmax);
        points.push_back(center);
        auto transformed = transform_points(points, camera_pose);
        viewer.addPolygon<pcl::PointXYZ>(Vector3Type_to_PointCloud(transformed), prefix + "negative_x");
    }

    {
        points.clear();
        points.push_back(maxmax);
        points.push_back(minmax);
        points.push_back(center);
        auto transformed = transform_points(points, camera_pose);
        viewer.addPolygon<pcl::PointXYZ>(Vector3Type_to_PointCloud(transformed), prefix + "positive_y");
    }
}

void initialize_visualizer(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    {
        pcl::PointXYZ p(1.0, 0.0, 0.0);
        viewer.addSphere(p, 0.2, "sphere_x", 0);
    }
    {
        pcl::PointXYZ p(0.0, 1.0, 0.0);
        viewer.addSphere(p, 0.1, "sphere_y", 0);
    }
}

}
