#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <sstream>
#include <random>
#include <functional>

#include <system-config.hpp>
#include <os/mutex.hpp>
#include <os/event.hpp>
#include <base/visualization.hpp>
#include <base/math.hpp>

using namespace std;

void
viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    mvSLAM::initialize_visualizer(viewer);

    // draw a sphere
    pcl::PointXYZ p;
    p.x = 1.0;
    p.y = 0.0;
    p.z = 0.0;
    viewer.addSphere(p, 0.25, "sphere", 0);

    // draw a polyhedron to represent a camera pose
    mvSLAM::add_camera_representation(mvSLAM::SE3(), "camera", viewer);

    std::cout<<"PCLVisualizer initialized."<<std::endl;
}

pcl::PointXYZRGB
get_random_point_xyzrgb()
{
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> uniform_dist(0, 255);
    static std::normal_distribution<mvSLAM::ScalarType> normal_dist(0.0, 1.0);
    uint8_t r = uniform_dist(generator); 
    uint8_t g = uniform_dist(generator); 
    uint8_t b = uniform_dist(generator); 
    mvSLAM::ScalarType x = normal_dist(generator);
    mvSLAM::ScalarType y = normal_dist(generator);
    mvSLAM::ScalarType z = normal_dist(generator);
    pcl::PointXYZRGB p(r, g, b);
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

pcl::PointXYZ
get_random_point_xyz()
{
    static std::default_random_engine generator;
    static std::normal_distribution<mvSLAM::ScalarType> normal_dist(0.0, 1.0);
    mvSLAM::ScalarType x = normal_dist(generator);
    mvSLAM::ScalarType y = normal_dist(generator);
    mvSLAM::ScalarType z = normal_dist(generator);
    pcl::PointXYZ p(x, y, z);
    std::cout<<"("<<p.x<<", "<<p.y<<", "<<p.z<<")"<<std::endl;
    return p;
}

int main()
{
    pcl::visualization::PCLVisualizer viewer("PCL Visualizer");
    viewerOneOff(viewer);

    const int point_count = 2000;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < point_count; ++i)
    {
        pc->push_back(get_random_point_xyzrgb());
    }
    viewer.addPointCloud<pcl::PointXYZRGB>(pc, "example points");
    //(void) pc;


    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
