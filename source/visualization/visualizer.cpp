#include <base/debug.hpp>
#include <base/parameter-manager.hpp>
#include <os/mutex.hpp>
#include <visualization/data-types.hpp>
#include <visualization/visualizer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <atomic>
#include <thread>

namespace mvSLAM
{
static const std::string module_name("Visualizer");
static Logger logger("[Visualizer]", true);

// conversion functions
static pcl::PointXYZ
mvSLAM_to_pcl(const VisualizationTypes::Point3 &p)
{
    return pcl::PointXYZ(p[0], p[1], p[2]);
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr
mvSLAM_to_pcl(const VisualizationTypes::PointCloud &pc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : pc)
    {
        result->push_back(mvSLAM_to_pcl(p.second));
    }
    return result;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr
mvSLAM_to_pcl(const std::vector<VisualizationTypes::Point3> &pc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : pc)
    {
        result->push_back(mvSLAM_to_pcl(p));
    }
    return result;
}

// id strings
static std::string
get_point_cloud_id_string(VisualizationTypes::PointCloudId id)
{
    static const std::string prefix("point_cloud ");
    return prefix + std::to_string(id);
}

static std::string
get_camera_id_string(VisualizationTypes::CameraId id)
{
    static const std::string prefix("camera ");
    return prefix + std::to_string(id);
}

/// Data container for @ref Visualizer.
struct VisualizerImpl
{
    const std::string name;
    const Visualizer::Params params;
    pcl::visualization::PCLVisualizer viewer;
    std::atomic<bool> viewer_thread_should_exit;

    Mutex mutex;  // protect everything down below
    std::thread viewer_thread;
    std::unordered_map<VisualizationTypes::PointCloudId,
            std::string> point_cloud_id_to_string;
    std::unordered_map<VisualizationTypes::CameraId,
            std::string> camera_id_to_string;

    VisualizerImpl(const std::string &n, const Visualizer::Params &p):
        name(n),
        params(p),
        viewer(),
        viewer_thread_should_exit(false),
        viewer_thread(),
        point_cloud_id_to_string(),
        camera_id_to_string()
    {
    }

    ~VisualizerImpl() = default;
};

static void
run_viewer_thread(VisualizerImpl *v)
{
    logger.info("thread start");
    while (!v->viewer_thread_should_exit.load() &&
           !v->viewer.wasStopped())
    {
        // FIXME: temp fix for race conditions. really all the modifications should
        // happen in the same thread context. That means we need to have an async
        // queue, and add items to the queue in Visualizer API's, and pop and process
        // items here.
        Lock lock(v->mutex);
        v->viewer.spinOnce(v->params.view_update_interval_ms);
    }
    // overwrite to make sure everything is consistent
    v->viewer_thread_should_exit = true;
    logger.info("thread stop");
}

Visualizer::Params
Visualizer::get_default_params()
{
    Params p;

    p.view_update_interval_ms = ParameterManager::get_value(
            module_name, "view_update_interval_ms", 100);

    return p;
}

Visualizer::Visualizer(const std::string &name, const Params &params):
    m_impl(new VisualizerImpl(name, params))
{
    logger.info("constructor, ", name);

    Lock lock(m_impl->mutex);

    m_impl->viewer.setBackgroundColor(0, 0, 0); // black
    m_impl->viewer.addCoordinateSystem(1.0); // coordinate system at origin
    m_impl->viewer.initCameraParameters();

    m_impl->viewer_thread = std::thread(run_viewer_thread, m_impl);
}

Visualizer::~Visualizer()
{
    // m_impl's dtor is called after this dtor
    logger.info("desctructor");
    // terminate viewer thread and wait
    m_impl->viewer_thread_should_exit = true;
    m_impl->viewer_thread.join();
}

void
Visualizer::set_camera_pose(VisualizationTypes::CameraId id,
        const Transformation &T_camera_to_world,
        ScalarType scale)
{
    assert(scale > 0);

    Lock lock(m_impl->mutex);
    std::string id_string;
    if (m_impl->camera_id_to_string.count(id) == 0) // a new camera
    {
        id_string = get_camera_id_string(id);
        m_impl->camera_id_to_string.emplace(id, id_string);
    }
    else
    {
        id_string = m_impl->camera_id_to_string.at(id);
        // delete
        m_impl->viewer.removeShape(id_string + " X");
        m_impl->viewer.removeShape(id_string + " Y");
        m_impl->viewer.removeShape(id_string + " Z");
    }

    {
        const auto origin = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, 0.0));
        const auto X = mvSLAM_to_pcl(T_camera_to_world * Point3(scale, 0.0, 0.0));
        double r = 1.0, g = 0.0, b = 0.0;
        m_impl->viewer.addLine(origin, X, r, g, b, id_string + " X");
    }
    
    {
        const auto origin = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, 0.0));
        const auto Y = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, scale, 0.0));
        double r = 0.0, g = 1.0, b = 0.0;
        m_impl->viewer.addLine(origin, Y, r, g, b, id_string + " Y");
    }

    {
        const auto origin = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, 0.0));
        const auto Z = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, scale));
        double r = 0.0, g = 0.0, b = 1.0;
        m_impl->viewer.addLine(origin, Z, r, g, b, id_string + " Z");
    }
}

template <typename PointCloudType>
static void
__set_point_cloud(VisualizerImpl *m_impl,
        VisualizationTypes::PointCloudId id,
        const PointCloudType &pc)
{
    auto pcl_point_cloud = mvSLAM_to_pcl(pc);

    Lock lock(m_impl->mutex);
    std::string id_string;
    if (m_impl->point_cloud_id_to_string.count(id) == 0) // new point cloud
    {
        id_string = get_point_cloud_id_string(id);
        m_impl->point_cloud_id_to_string.insert({id, id_string});
        m_impl->viewer.addPointCloud<pcl::PointXYZ>(pcl_point_cloud, id_string);
    }
    else // existing point cloud
    {
        id_string = m_impl->point_cloud_id_to_string.at(id);
        m_impl->viewer.updatePointCloud<pcl::PointXYZ>(pcl_point_cloud, id_string);
    }
}

void
Visualizer::set_point_cloud(VisualizationTypes::PointCloudId id,
        const VisualizationTypes::PointCloud &pc)
{
    __set_point_cloud(m_impl, id, pc);
}

void
Visualizer::set_point_cloud(VisualizationTypes::PointCloudId id,
        const std::vector<VisualizationTypes::Point3> &pc)
{
    __set_point_cloud(m_impl, id, pc);
}

bool
Visualizer::is_window_closed()
{
    Lock lock(m_impl->mutex);
    return m_impl->viewer.wasStopped();
}

}
