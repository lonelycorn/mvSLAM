#include "unit-test.hpp"
#include "unit-test-helper.hpp"
#include <math/kalman-filter.hpp>

using namespace unit_test;

//#define DEBUG_OUTPUT

struct MovingMass
{
    MovingMass():
        position(0), velocity(0)
    {
    }

    mvSLAM::ScalarType
    get_position_measurement(mvSLAM::ScalarType noise_stddev)
    {
        return position + get_gaussian(0, noise_stddev);
    }

    mvSLAM::ScalarType
    get_velocity_measurement(mvSLAM::ScalarType noise_stddev)
    {
        return velocity + get_gaussian(0, noise_stddev);
    }

    void update(mvSLAM::ScalarType v, mvSLAM::ScalarType noise_stddev, mvSLAM::ScalarType dt)
    {
        auto acc = get_gaussian(0, noise_stddev);
        auto new_velocity = v + acc * dt; 
        position += (new_velocity + velocity) * dt / 2;
        velocity = new_velocity;
    }

    mvSLAM::ScalarType position;
    mvSLAM::ScalarType velocity;
};

UNIT_TEST(kalman_filter_moving_mass)
{
    constexpr mvSLAM::ScalarType tolerance = 2e-2;

    constexpr mvSLAM::ScalarType dt = 1.0;
    constexpr int state_size = 2;
    constexpr int measurement_size = 1;
    constexpr int input_size = 1;
    constexpr mvSLAM::ScalarType process_noise_stddev = 1e-2;
    constexpr mvSLAM::ScalarType measurement_noise_stddev = 1e-2;
    constexpr int step_count = 100;
    constexpr mvSLAM::ScalarType velocity = 1.0;

    // real system
    MovingMass mm;

    mvSLAM::KalmanFilter<state_size> kf;

    // state transition
    mvSLAM::KalmanFilter<state_size>::TransitionMatrixType transition_matrix;
    transition_matrix << 1, dt,
                         0, 1;
    mvSLAM::KalmanFilter<state_size>::ProcessNoiseType process_noise;
    process_noise << mvSLAM::sqr(process_noise_stddev), 0,
                     0, mvSLAM::sqr(process_noise_stddev);

    // external input (constant zero)
    mvSLAM::KalmanFilter<state_size>::InputMatrixType<input_size> input_matrix;
    input_matrix << 0, 0;
    mvSLAM::KalmanFilter<state_size>::InputType<input_size> input;
    input << 0;

    // position sensor
    mvSLAM::KalmanFilter<state_size>::MeasurementMatrixType<measurement_size> measurement_matrix;
    measurement_matrix << 1, 0;
    mvSLAM::KalmanFilter<state_size>::MeasurementCovarType<measurement_size> measurement_covar;
    measurement_covar << mvSLAM::sqr(measurement_noise_stddev);
    mvSLAM::KalmanFilter<state_size>::MeasurementType<measurement_size> measurement;

    // initialize kalman filter
    mvSLAM::KalmanFilter<state_size>::StateType init_state;
    init_state << mm.get_position_measurement(measurement_noise_stddev),
                  mm.get_velocity_measurement(measurement_noise_stddev);

    mvSLAM::KalmanFilter<state_size>::CovarType init_covar;
    init_covar << 1e3, 0,
                  0, 1e3;

    {
        bool success = kf.init(init_state, init_covar);
        ASSERT_TRUE(success);
    }

    for (int i = 0; i < step_count; ++i)
    {
        mm.update(velocity, process_noise_stddev, dt);

        measurement[0] = mm.get_position_measurement(measurement_noise_stddev);
        
        {
            bool success = kf.process_update(transition_matrix, process_noise, input_matrix, input);
            ASSERT_TRUE(success);
        }

        {
            bool success = kf.measurement_update(measurement_matrix, measurement, measurement_covar);
            ASSERT_TRUE(success);
        }
#ifdef DEBUG_OUTPUT
        std::cout << "\n===== i = " << i << " =====" << std::endl;
        std::cout << "final state =\n" << kf.get_state() << std::endl;
        std::cout << "final covar =\n" << kf.get_covar() << std::endl;
#endif
    }

#ifdef DEBUG_OUTPUT
    std::cout << "true position = " << mm.position << ", true velocity = " << mm.velocity << std::endl;
#endif

    auto final_state = kf.get_state();

    ASSERT_EQUAL(final_state[0], mm.position, tolerance);
    ASSERT_EQUAL(final_state[1], mm.velocity, tolerance);

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
    return 0;
}
