#pragma once
#include <front-end/data-type.hpp>
#include <os/mutex.hpp>
#include <unordered_map>

namespace mvSLAM
{

/** A global singleton that owns all image frames.
 */
class FrameManager
{
public:
    static FrameManager &get_instance();

    FrontEndTypes::FrameId
        add_frame(timestamp_us_t capture_time, const ImageGrayscale &image);

    /** Erase the sepcified frame.
     * @return false if @p id is invalid.
     */
    bool erase_frame(FrontEndTypes::FrameId id);

    /** Get the frame for the specified id.
     * @return nullptr if @p id is invalid.
     */
    FrontEndTypes::FramePtr
        get_frame(FrontEndTypes::FrameId id) const;

    /// Get the number of frames on record.
    size_t size() const;

private:
    FrameManager();
    ~FrameManager();

    /** Check if @p id is registered.
     * Must lock @ref mutex;
     */
    bool check_valid_id(FrontEndTypes::FrameId id) const;

    mutable Mutex m_mutex;
    std::unordered_map<FrontEndTypes::FrameId,
        FrontEndTypes::FramePtr> m_frame_map;
};

}
