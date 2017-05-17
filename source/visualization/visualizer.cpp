#include <base/debug.hpp>
#include <base/parameter-manager.hpp>
#include <os/mutex.hpp>
#include <visualization/data-types.hpp>
#include <visualization/visualizer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <atomic>
#include <thread>
#include <random>

namespace mvSLAM
{
static const std::string module_name("Visualizer");
static Logger logger("[Visualizer]", true);

// conversion functions for pcl::PointXYZ
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

#if 0
// conversion functions for pcl::PointXYZRGB
static pcl::PointXYZRGB
mvSLAM_to_pcl(const VisualizationTypes::Point3 &p,
              ScalarType r, ScalarType g, ScalarType b)
{
    constexpr ScalarType max_intensity(255);
    pcl::PointXYZRGB result;
    result.r = static_cast<uint8_t>(r * max_intensity);
    result.g = static_cast<uint8_t>(g * max_intensity);
    result.b = static_cast<uint8_t>(b * max_intensity);
    result.x = p[0];
    result.y = p[1];
    result.z = p[2];
    return result;
}

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
mvSLAM_to_pcl(const VisualizationTypes::PointCloud &pc,
              ScalarType r, ScalarType g, ScalarType b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &p : pc)
    {

        result->push_back(mvSLAM_to_pcl(p.second, r, g, b));
    }
    return result;
}

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
mvSLAM_to_pcl(const std::vector<VisualizationTypes::Point3> &pc,
              ScalarType r, ScalarType g, ScalarType b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &p : pc)
    {
        result->push_back(mvSLAM_to_pcl(p, r, g, b));
    }
    return result;
}
#endif

/// get random number between 0 and 1. TODO: move to base/random
static ScalarType
get_random_number()
{
    static std::default_random_engine generator;
    std::uniform_real_distribution<ScalarType> distribution(0.0, 1.0);
    return distribution(generator);
}

struct MetaDataPointCloud
{
    VisualizationTypes::PointCloudId id;
    std::string name;
    ScalarType r, g, b;
    MetaDataPointCloud(VisualizationTypes::PointCloudId id_):
        id(id_),
        name("point_cloud " + std::to_string(id)),
        r(get_random_number()),
        g(get_random_number()),
        b(get_random_number())
    {
        logger.info("id = ", id, ", r = ", r, ", g = ", g, ", b = ", b);
    }
};

struct MetaDataCamera
{
    VisualizationTypes::CameraId id;
    std::string name_X, name_Y, name_Z;
    MetaDataCamera(VisualizationTypes::CameraId id_):
        id(id_),
        name_X("camera " + std::to_string(id) + " X"),
        name_Y("camera " + std::to_string(id) + " Y"),
        name_Z("camera " + std::to_string(id) + " Z")
    {
    }
};

/// Data container for @ref Visualizer.
struct VisualizerImpl
{
    const std::string name;
    const Visualizer::Params params;
    std::atomic<bool> viewer_thread_should_exit;

    Mutex mutex;  // protect everything down below
    pcl::visualization::PCLVisualizer viewer;
    std::thread viewer_thread;
    std::unordered_map<VisualizationTypes::PointCloudId,
            MetaDataPointCloud> meta_data_point_cloud;
    std::unordered_map<VisualizationTypes::CameraId,
            MetaDataCamera> meta_data_camera;

    VisualizerImpl(const std::string &n, const Visualizer::Params &p):
        name(n),
        params(p),
        viewer_thread_should_exit(false),
        mutex(),
        viewer(),
        viewer_thread(),
        meta_data_point_cloud(),
        meta_data_camera()
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

    p.point_size_pixel = ParameterManager::get_value(
            module_name, "point_size_pixel", 4);

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
    if (m_impl->meta_data_camera.count(id) == 0) // a new camera
    {
        m_impl->meta_data_camera.emplace(id, MetaDataCamera(id));
    }
    else
    {
        const auto &m = m_impl->meta_data_camera.at(id);
        // delete
        m_impl->viewer.removeShape(m.name_X);
        m_impl->viewer.removeShape(m.name_Y);
        m_impl->viewer.removeShape(m.name_Z);
    }

    const auto &m = m_impl->meta_data_camera.at(id);
    const auto origin = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, 0.0));
    {
        const auto X = mvSLAM_to_pcl(T_camera_to_world * Point3(scale, 0.0, 0.0));
        double r = 1.0, g = 0.0, b = 0.0;
        m_impl->viewer.addLine(origin, X, r, g, b, m.name_X);
    }
    
    {
        const auto Y = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, scale, 0.0));
        double r = 0.0, g = 1.0, b = 0.0;
        m_impl->viewer.addLine(origin, Y, r, g, b, m.name_Y);
    }

    {
        const auto Z = mvSLAM_to_pcl(T_camera_to_world * Point3(0.0, 0.0, scale));
        double r = 0.0, g = 0.0, b = 1.0;
        m_impl->viewer.addLine(origin, Z, r, g, b, m.name_Z);
    }
}

template <typename PointCloudType>
static void
__set_point_cloud(VisualizerImpl *m_impl,
        VisualizationTypes::PointCloudId id,
        const PointCloudType &pc)
{
    const auto pcl_point_cloud = mvSLAM_to_pcl(pc);

    Lock lock(m_impl->mutex);
    if (m_impl->meta_data_point_cloud.count(id) == 0) // new point cloud
    {
        m_impl->meta_data_point_cloud.emplace(id, MetaDataPointCloud(id));
        const auto &m = m_impl->meta_data_point_cloud.at(id);
        const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            color_handler(pcl_point_cloud, m.r * 255, m.g * 255, m.b * 255);
        m_impl->viewer.addPointCloud<pcl::PointXYZ>(pcl_point_cloud,
                color_handler, m.name);
        m_impl->viewer.setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                m_impl->params.point_size_pixel, m.name);
    }
    else // existing point cloud
    {
        const auto &m = m_impl->meta_data_point_cloud.at(id);
        const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            color_handler(pcl_point_cloud, m.r * 255, m.g * 255, m.b * 255);
        m_impl->viewer.updatePointCloud<pcl::PointXYZ>(pcl_point_cloud,
                color_handler, m.name);
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
