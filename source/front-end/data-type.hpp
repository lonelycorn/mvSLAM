#pragma once
#include <system-config.hpp>
#include <base/space.hpp>
#include <vision/visual-feature.hpp>
#include <os/time.hpp>

#include <memory>

namespace mvSLAM
{

struct FrontEndTypes
{
    using FrameId = Id::Type;
    static FrameId generate_frame_id();

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

    using FramePtr = std::shared_ptr<Frame>;
};



};
