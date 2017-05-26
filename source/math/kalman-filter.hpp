#pragma once
#include <system-config.hpp>

#include <cstdlib>
#include <Eigen/Core>

namespace mvSLAM
{
template <int STATE_SIZE>
class KalmanFilter
{
public:
    static_assert(STATE_SIZE > 0, "state must contain at least 1 variable");

    using StateType = Eigen::Matrix<ScalarType, STATE_SIZE, 1>;
    using CovarType = Eigen::Matrix<ScalarType, STATE_SIZE, STATE_SIZE>;

    using TransitionMatrixType = Eigen::Matrix<ScalarType, STATE_SIZE, STATE_SIZE>;
    using ProcessNoiseType = Eigen::Matrix<ScalarType, STATE_SIZE, STATE_SIZE>;

    template <int INPUT_SIZE>
    using InputType = Eigen::Matrix<ScalarType, INPUT_SIZE, 1>;
    template <int INPUT_SIZE>
    using InputMatrixType = Eigen::Matrix<ScalarType, STATE_SIZE, INPUT_SIZE>;

    template <int MEASUREMENT_SIZE>
    using MeasurementType = Eigen::Matrix<ScalarType, MEASUREMENT_SIZE, 1>;
    template <int MEASUREMENT_SIZE>
    using MeasurementMatrixType = Eigen::Matrix<ScalarType, MEASUREMENT_SIZE, STATE_SIZE>;
    template <int MEASUREMENT_SIZE>
    using MeasurementCovarType = Eigen::Matrix<ScalarType, MEASUREMENT_SIZE, MEASUREMENT_SIZE>;

    KalmanFilter():
        m_initialized(false),
        m_state(),
        m_covar()
    {
    }

    ~KalmanFilter() = default;

    /// return true if successfully initialized; false otherwise, and original states are not changed.
    bool init(const StateType &init_state,
              const CovarType &init_covar)
    {
        StateType original_state = m_state;
        CovarType original_covar = m_covar;

        m_state = init_state;
        m_covar = init_covar;

        if (check_state_covar_sanity())
        {
            m_initialized = true;
        }
        else
        {
            // revert changes
            m_initialized = false;
            std::swap(original_state, m_state);
            std::swap(original_covar, m_covar);
        }
        return m_initialized;
    }

    const StateType &get_state() const
    {
        assert(m_initialized);
        return m_state;
    }

    const CovarType &get_covar() const
    {
        assert(m_initialized);
        return m_covar;
    }

    /** Process update driven by noise.
     * @return true if successfully updated; false otherwise, and states are not changed.
     */
    bool process_update(const TransitionMatrixType &transition_matrix,
                        const CovarType &process_noise_covar)
    {
        assert(m_initialized);
        assert(check_state_covar_sanity());

        // create aliases
        auto &x = m_state;
        auto &P = m_covar;
        auto &F = transition_matrix;
        auto &Q = process_noise_covar;

        // save original states
        StateType original_x = x;
        CovarType original_P = P;

        // process update
        x = F * x;
        P = F * P * F.transpose() + Q;

        if (!check_state_covar_sanity())
        {
            // revert changes
            std::swap(x, original_x);
            std::swap(P, original_P);
            return false;
        }
        else
        {
            return true;
        }

    }

    /** Process update
     * @return true if successfully updates; otherwise false, and the original states don't change
     */
    template <int INPUT_SIZE>
    bool process_update(const TransitionMatrixType &transition_matrix,
                        const CovarType &process_noise_covar,
                        const InputMatrixType<INPUT_SIZE> &input_matrix,
                        const InputType<INPUT_SIZE> &input)
    {
        static_assert(INPUT_SIZE > 0, "input must contain at least 1 variable");
        assert(m_initialized);
        assert(check_state_covar_sanity());

        // create aliases
        auto &x = m_state;
        auto &P = m_covar;
        auto &B = input_matrix;
        auto &u = input;

        // save original states
        StateType original_x = x;
        CovarType original_P = P;

        // process update w/o input
        if (!process_update(transition_matrix, process_noise_covar))
        {
            return false;
        }

        // add external input
        // NOTE: the effects of the external input are deterministic, thus no
        //       change on the covar
        x = x + B * u;

        if (!check_state_covar_sanity())
        {
            // revert changes
            std::swap(x, original_x);
            std::swap(P, original_P);
            return false;
        }
        else
        {
            return true;
        }

    }

    /** measurement update.
     * @return true if successfully updated; otherwise false, and the original states don't change.
     */
    template <int MEASUREMENT_SIZE>
    bool measurement_update(const MeasurementMatrixType<MEASUREMENT_SIZE> &measurement_matrix,
                            const MeasurementType<MEASUREMENT_SIZE> &measurement,
                            const MeasurementCovarType<MEASUREMENT_SIZE> &measurement_covar)
    {
        static_assert(MEASUREMENT_SIZE > 0, "measurement must contain at least 1 variable");
        assert(m_initialized);
        assert(check_state_covar_sanity());

        // create aliases
        auto &x = m_state;
        auto &P = m_covar;
        auto &H = measurement_matrix;
        auto &y = measurement;
        auto &R = measurement_covar;

        // save original states
        StateType original_x = x;
        CovarType original_P = P;

        // measurement update
        Eigen::Matrix<ScalarType, MEASUREMENT_SIZE, MEASUREMENT_SIZE>
            S = H * P * H.transpose() + R;// innovation covar
        Eigen::Matrix<ScalarType, STATE_SIZE, MEASUREMENT_SIZE>
            K = P * H.transpose() * S.inverse(); // NOTE: we may want to optimize for speed by avoid computing the inverse explicitly.
        x = x + K * (y - H * x);
        P = P - K * H * P;

        if (!check_state_covar_sanity())
        {
            // revert changes
            std::swap(x, original_x);
            std::swap(P, original_P);
            return false;
        }
        else
        {
            return true;
        }
    }

private:

    /// check if state and covar are finite values
    bool check_state_covar_sanity() const
    {
        for (int i = 0; i < STATE_SIZE; ++i)
        {
            if (!std::isfinite(m_state[i]))
            {
                return false;
            }
            for (int j = 0; j < STATE_SIZE; ++j)
            {
                if (!std::isfinite(m_covar(i, j)))
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool m_initialized;
    StateType m_state;
    CovarType m_covar;
};

}
