#pragma once
#include <system-config.hpp>

namespace mvSLAM
{

template <typename ValueType>
struct LowPassFilter
{
    /** Ctor.
     * @param [in] init_value   initial value
     * @param [in] low_pass_gain    smoothing factor between 0 and 1.
     *      higher value leads to less smoothing.
     */
    LowPassFilter(const ValueType &init_value,
                  ScalarType low_pass_gain):
        m_value(init_value),
        m_gain(low_pass_gain)
    {
    }

    const ValueType &get_value() const
    {
        return m_value;
    }

    const ValueType &update(const ValueType &new_value)
    {
        m_value = m_gain * new_value + (1 - m_gain) * m_value;
        return m_value;
    }
private:
    ValueType m_value;
    ScalarType m_gain;
};

}
