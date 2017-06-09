#pragma once
#include <base/data-type.hpp>
#include <vision/visual-feature.hpp>
#include <os/time.hpp>

#include <memory>

namespace mvSLAM
{

struct FrontEndTypes
{
    using FrameId = Id::Type;
    static FrameId generate_frame_id();

    // TODO: make const?
    // TODO: only allow FrameManager to create
    /// data structure to store basic information useful to front-end.
    struct Frame
    {
        FrameId id;
        timestamp_us_t capture_time;
        VisualFeature visual_feature;

        // for debugging
        ImageGrayscale image;

        /// Ctor
        //Frame();
        Frame(FrameId id_,
              timestamp_us_t capture_time_,
              const VisualFeature &visual_feature_,
              const ImageGrayscale &image_);
    };

    // by default, no write access
    using FramePtr = std::shared_ptr<const Frame>;
};



};
