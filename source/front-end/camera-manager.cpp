#include <os/mutex.hpp>
#include <base/debug.hpp>
#include <front-end/camera-manager.hpp>

namespace mvSLAM
{
Logger logger("[CameraManager]", true);

// default camera
PinholeCamera m_camera(Matrix3Type::Identity(), SE3());
Mutex m_mutex;

const PinholeCamera &
CameraManager::get_camera()
{
    Lock lock(m_mutex);
    return m_camera;
}

void
CameraManager::load_from_file(const std::string &filename)
{
    Lock lock(m_mutex);
    logger.info("load from '", filename, "'");
    bool success = m_camera.load_from_file(filename);
    assert(success);
    logger.debug("camera intrinsics:\n", m_camera.get_intrinsics(),
                 "\ncamera extrinsics:\n", m_camera.get_extrinsics());
}

void
CameraManager::save_to_file(const std::string &filename)
{
    Lock lock(m_mutex);
    logger.info("save to '", filename, "'");
    bool success = m_camera.save_to_file(filename);
    assert(success);
}
}
