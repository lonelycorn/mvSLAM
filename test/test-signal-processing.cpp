#include "unit-test.hpp"
#include <math/signal-processing.hpp>
#include <vector>
using namespace unit_test;

UNIT_TEST(low_pass_filter)
{
    const mvSLAM::ScalarType tolerance = 1e-3;
    const mvSLAM::ScalarType lpf_gain = 0.4;
    const std::vector<mvSLAM::ScalarType> value{0, 1, 2, 3, 4};
    const std::vector<mvSLAM::ScalarType> value_lpf{0, 0.4, 1.04, 1.824, 2.6944};
    mvSLAM::LowPassFilter<mvSLAM::ScalarType> lpf(value[0], lpf_gain);
    for (size_t i = 1; i < value.size(); ++i)
    {
        auto result = lpf.update(value[i]);
        ASSERT_EQUAL(result, value_lpf[i], tolerance);
    }
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}

