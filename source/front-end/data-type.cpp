#include <front-end/data-type.hpp>

#include <atomic>

namespace mvSLAM
{
FrontEndTypes::FrameId
FrontEndTypes::generate_frame_id()
{
    static std::atomic<FrameId> next_id(0);
    return next_id.fetch_add(1);
}

/*
FrontEndTypes::Frame::Frame():
    id(Id::INVALID),
    capture_time(0),
    visual_feature(),
    image()
{
}
*/
FrontEndTypes::Frame::Frame(FrameId id_,
                            timestamp_us_t capture_time_,
                            const VisualFeature &visual_feature_,
                            const ImageGrayscale &image_):
    id(id_),
    capture_time(capture_time_),
    visual_feature(visual_feature_),
    image(image_)
{
}

}
