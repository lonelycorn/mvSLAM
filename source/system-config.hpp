#pragma once
#include <opencv2/opencv.hpp>
#include <limits>
namespace mvSLAM
{
using ScalarType = double;
constexpr ScalarType epsilon = std::numeric_limits<ScalarType>::epsilon();
constexpr ScalarType tolerance = epsilon * 10;
/// A value large enough to be treated as infinity; not to be confused with
constexpr ScalarType infinity = std::numeric_limits<ScalarType>::max() / 10;
}
