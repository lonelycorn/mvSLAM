#include <front-end/frame-manager.hpp>
#include <base/debug.hpp>

namespace mvSLAM
{
static Logger logger("FrameManager");

FrameManager &
FrameManager::get_instance()
{
    static FrameManager instance;
    return instance;
}

FrontEndTypes::FrameId
FrameManager::add_frame(timestamp_us_t capture_time, const ImageGrayscale &image)
{
    logger.info("add_frame, capture_time = ", capture_time);
    FrontEndTypes::FrameId id = FrontEndTypes::generate_frame_id();

    // extract information into a frame
    VisualFeature vf = VisualFeature::extract(image);
    logger.debug("add_frame, id = ", id, ", visual feature size = ", vf.size());

    {
        Lock lock(m_mutex);
        m_frame_map.emplace(id,
                std::make_shared<FrontEndTypes::Frame>(id, capture_time, vf, image));
    }

    return id;
}

bool
FrameManager::erase_frame(FrontEndTypes::FrameId id)
{
    logger.info("erase_frame, id = ", id);
    Lock lock(m_mutex);
    if (!check_valid_id(id))
    {
        logger.error("erase_frame, invalid id = ", id);
        assert(false);
        return false;
    }
    m_frame_map.erase(id);
    return true;
}

FrontEndTypes::FramePtr
FrameManager::get_frame(FrontEndTypes::FrameId id) const
{
    Lock lock(m_mutex);
    if (!check_valid_id(id))
    {
        logger.error("get_frame, invalid id = ", id);
        return nullptr;
    }
    return m_frame_map.at(id);
}

size_t
FrameManager::size() const
{
    Lock lock(m_mutex);
    return m_frame_map.size();
}

FrameManager::FrameManager():
    m_mutex(),
    m_frame_map()
{
    logger.info("constructor");
}

FrameManager::~FrameManager()
{
    logger.info("destructor");
}

bool
FrameManager::check_valid_id(FrontEndTypes::FrameId id) const
{
    return ((id != Id::INVALID) && (m_frame_map.count(id) > 0));
}

}

