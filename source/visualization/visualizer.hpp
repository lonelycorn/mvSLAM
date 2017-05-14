#pragma once
#include <base/space.hpp>
#include <os/time.hpp>
#include <visualization/data-types.hpp>

#include <string>
#include <vector>

namespace mvSLAM
{

/// Forwared declaration
class VisualizerImpl;

class Visualizer
{
public:
    struct Params
    {
        timestamp_ms_t view_update_interval_ms;
    };

    static Params get_default_params();

    Visualizer(const std::string &name, const Params &params);
    ~Visualizer();
    
    /** Visualize camera pose.
     * @param [in] id   unique ID of the camera.
     * @param [in] T_camera_to_world transformation from the camera ref
     *      frame to world ref frame.
     * @param [in] scale    scale of the camera symbol.
     */
    void set_camera_pose(VisualizationTypes::CameraId id,
            const Transformation &T_camera_to_world,
            ScalarType scale = 1.0);

    /** Visualize point cloud.
     * @param [in] id   unique ID of the point cloud.
     * @param [in] pc   point positions in world ref frame.
     */
    void set_point_cloud(VisualizationTypes::PointCloudId id,
            const VisualizationTypes::PointCloud &pc);

    /// overload for std::vector
    void set_point_cloud(VisualizationTypes::PointCloudId id,
            const std::vector<VisualizationTypes::Point3> &pc);
    
    bool is_window_closed();
private:
    VisualizerImpl *m_impl;
};

}
