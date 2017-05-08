#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <sstream>
#include <random>
#include <functional>

#include <system-config.hpp>
#include <os/mutex.hpp>
#include <os/event.hpp>
#include <base/error.hpp>
#include <base/visualization.hpp>
#include <base/math.hpp>

using namespace std;
int value_shared;
mvSLAM::Mutex mutex;

mvSLAM::Mutex event_mutex;
mvSLAM::Event event;
bool ready = false;

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
    {
        mvSLAM::Lock lock(event_mutex);
        ready = true;
        event.trigger_all();
    }
}

void
viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    std::stringstream ss;
    int value;
    {
        mvSLAM::Lock lock(mutex);
        value = value_shared;
    }
    ss<<"[Once per viewer loop] value = "<<value;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    mvSLAM::Lock lock(mutex);
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
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    const int point_count = 2000;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < point_count; ++i)
    {
        pc->push_back(get_random_point_xyzrgb());
    }
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < point_count; ++i)
    {
        pc->push_back(get_random_point_xyz());
    }
    */
    viewer.showCloud(pc);


    // called once when starting the thread
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // called every iteration 
    viewer.runOnVisualizationThread(viewerPsycho);

    {
        mvSLAM::Lock lock(event_mutex);
        while (!ready)
            event.wait(event_mutex);
    }
    while (!viewer.wasStopped())
    {
        int value;
        std::cout<<"input value = ";
        std::cin>>value;
        {
            mvSLAM::Lock lock(mutex);
            value_shared = value;
        }
    }

    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}
