#pragma once
#include <opencv2/opencv.hpp>
#include <limits>

namespace mvSLAM
{
using ScalarType = double;
/// machine precision
constexpr ScalarType epsilon = std::numeric_limits<ScalarType>::epsilon();
/// default tolerance when checking equality
constexpr ScalarType tolerance = epsilon * 1000;
/// threshold for approximation with Taylor expansion
constexpr ScalarType taylor_threshold = static_cast<ScalarType>(1e-5);
/// A value large enough to be treated as infinity; not to be confused with
constexpr ScalarType infinity = std::numeric_limits<ScalarType>::max() / 10;
}
