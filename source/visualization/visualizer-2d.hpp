#pragma once
#include <os/time.hpp>
#include <visualization/data-type.hpp>
#include <string>

namespace mvSLAM
{

/// Forward declaration
class Visualizer2dImpl;

class Visualizer2d
{
public:
    struct Params
    {
        timestamp_ms_t view_update_interval_ms;
        bool view_update_immediately;
        uint32_t point_size_pixel;
        uint32_t line_width_pixel;
    };

    static Params get_default_params();

    Visualizer2d(const std::string &name, const Params &params);
    ~Visualizer2d();

    void set_key_frame(const VisualizationTypes::KeyFrame &key_frame);

    void set_matched_image_pair(const VisualizationTypes::MatchedImagePair &image_pair);

private:
    Visualizer2dImpl *m_impl;
};

}
