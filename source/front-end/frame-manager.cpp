#include <front-end/frame-manager.hpp>
#include <base/debug.hpp>
#include <math/kalman-filter.hpp>
#include <os/time.hpp>

namespace mvSLAM
{
//#define DEBUG_FPS
static Logger logger("[FrameManager]", false);

class FpsKalmanFilter
{
public:
    FpsKalmanFilter():
        m_kf(),
        m_initialized(false),
        m_update_time(0)
    {
    }

    void update(FrontEndTypes::FrameId id,
                timestamp_us_t capture_time)
    {
        ScalarType time = static_cast<ScalarType>(capture_time) / MS_PER_S;

        if (!m_initialized)
        {
            KfType::StateType state;
            KfType::CovarType covar;

            state << id, 0;
            covar << sqr(INITIAL_STATE_STDDEV), 0,
                     0, sqr(INITIAL_STATE_STDDEV);
            m_initialized = m_kf.init(state, covar);
            assert(m_initialized);
        }
        else
        {
            ScalarType dt = time - m_update_time;

            // process update
            KfType::TransitionMatrixType transition_matrix;
            transition_matrix << 1, dt,
                                 0, 1;
            KfType::ProcessNoiseType process_covar;
            process_covar << sqr(PROCESS_NOISE_STDDEV), 0,
                             0, sqr(PROCESS_NOISE_STDDEV);
            {
                bool success = m_kf.process_update(transition_matrix, process_covar);
                (void) success;
                assert(success);
            }
#ifdef DEBUG_FPS
            logger.debug("process update (dt = ", dt, ")\nstate =\n", m_kf.get_state(), "\ncovar =\n", m_kf.get_covar());
            logger.debug("F =\n", transition_matrix, "\nQ =\n", process_covar);
#endif

            // measurement update
            KfType::MeasurementMatrixType<MEASUREMENT_SIZE> measurement_matrix;
            KfType::MeasurementCovarType<MEASUREMENT_SIZE> measurement_covar;
            KfType::MeasurementType<MEASUREMENT_SIZE> measurement;
            measurement_covar << sqr(MEASUREMENT_NOISE_STDDEV);
            measurement << id;
            measurement_matrix << 1, 0;
            {
                bool success = m_kf.measurement_update(measurement_matrix,
                                                       measurement,
                                                       measurement_covar);
                (void) success;
                assert(success);
            }
#ifdef DEBUG_FPS
            logger.debug("measurement update (id = ", id, ")\nstate =\n", m_kf.get_state(), "\ncovar =\n", m_kf.get_covar());
            logger.debug("H =\n", measurement_matrix, "\nR =\n", measurement_covar);
#endif
        }
        m_update_time = time;
    }

    ScalarType get_fps() const
    {
        assert(m_initialized);
        const auto &state = m_kf.get_state();
        return state[1];
    }
private:
    static constexpr size_t STATE_SIZE = 2; // current frame time, frame rate
    static constexpr ScalarType PROCESS_NOISE_STDDEV = 1e-2; // NOTE: large value leads to slower convergence
    static constexpr ScalarType INITIAL_STATE_STDDEV = 1e0;
    static constexpr size_t MEASUREMENT_SIZE = 1;
    static constexpr ScalarType MEASUREMENT_NOISE_STDDEV = 1e-2;
    using KfType = KalmanFilter<STATE_SIZE>;

    KfType m_kf;
    bool m_initialized;
    ScalarType m_update_time;
};
constexpr size_t FpsKalmanFilter::STATE_SIZE;
constexpr ScalarType FpsKalmanFilter::INITIAL_STATE_STDDEV;
constexpr ScalarType FpsKalmanFilter::PROCESS_NOISE_STDDEV;
constexpr size_t FpsKalmanFilter::MEASUREMENT_SIZE;
constexpr ScalarType FpsKalmanFilter::MEASUREMENT_NOISE_STDDEV;

class FrameManagerImpl
{
public:
    FrontEndTypes::FrameId add_frame(timestamp_us_t capture_time,
                                     const ImageGrayscale &image)
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
            m_fps.update(id, capture_time);
        }

        return id;
    }

    bool erase_frame(FrontEndTypes::FrameId id)
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

    FrontEndTypes::FramePtr get_frame(FrontEndTypes::FrameId id) const
    {
        Lock lock(m_mutex);
        if (!check_valid_id(id))
        {
            logger.error("get_frame, invalid id = ", id);
            return nullptr;
        }
        return m_frame_map.at(id);
    }

    size_t size() const
    {
        Lock lock(m_mutex);
        return m_frame_map.size();
    }

    ScalarType get_fps() const
    {
        Lock lock(m_mutex);
        return m_fps.get_fps();
    }

    FrameManagerImpl():
        m_mutex(),
        m_frame_map()
    {
        logger.info("constructor");
    }
    ~FrameManagerImpl()
    {
        logger.info("destructor");
    }
private:
    /** Check if @p id is registered.
     * Must lock @ref mutex;
     */
    bool check_valid_id(FrontEndTypes::FrameId id) const
    {
        return ((id != Id::INVALID) && (m_frame_map.count(id) > 0));
    }

    mutable Mutex m_mutex;
    std::unordered_map<FrontEndTypes::FrameId,
        FrontEndTypes::FramePtr> m_frame_map;
    FpsKalmanFilter m_fps;
};

static FrameManagerImpl &get_impl()
{
    static FrameManagerImpl instance;
    return instance;
}

FrontEndTypes::FrameId
FrameManager::add_frame(timestamp_us_t capture_time, const ImageGrayscale &image)
{
    return get_impl().add_frame(capture_time, image);
}

bool
FrameManager::erase_frame(FrontEndTypes::FrameId id)
{
    return get_impl().erase_frame(id);
}

FrontEndTypes::FramePtr
FrameManager::get_frame(FrontEndTypes::FrameId id)
{
    return get_impl().get_frame(id);
}

size_t
FrameManager::size()
{
    return get_impl().size();
}

ScalarType
FrameManager::get_fps()
{
    return get_impl().get_fps();
}

}

