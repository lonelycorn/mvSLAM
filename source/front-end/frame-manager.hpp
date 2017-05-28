#pragma once
#include <front-end/data-type.hpp>
#include <math/signal-processing.hpp>
#include <os/mutex.hpp>
#include <unordered_map>

namespace mvSLAM
{

/** A global singleton that owns all image frames.
 */
class FrameManager
{
public:

    /** Register a new image.
     * @return a unique ID
     */
    static FrontEndTypes::FrameId add_frame(timestamp_us_t capture_time,
                                            const ImageGrayscale &image);

    /** Erase the sepcified frame.
     * @return false if @p id is invalid.
     */
    static bool erase_frame(FrontEndTypes::FrameId id);

    /** Get the frame for the specified id.
     * @return nullptr if @p id is invalid.
     */
    static FrontEndTypes::FramePtr get_frame(FrontEndTypes::FrameId id);

    /// Get the number of frames on record.
    static size_t size();

    /// Get frame rate estimate.
    static ScalarType get_fps();

};

}
