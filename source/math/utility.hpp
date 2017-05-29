#pragma once

namespace mvSLAM
{

/** square of the give value
 */
template <class T>
T sqr(const T &v)
{
    return v * v;
}

/** constrain the value with specified lower and upper bounds.
 */
template <class T>
T constrain(const T &v, const T &&lower, const T &&upper)
{
    return ((v < lower) ? lower : ((v > upper) ? upper : v));
}

/** before C++14, std::min is not constexpr
 */
template <class T>
constexpr const T &constexpr_min(const T &a, const T &b)
{
    return (a < b) ? a : b;
}

}
