#pragma once
#include <system-config.hpp>
#include <base/math.hpp>
#include <vector>

namespace unit_test
{

mvSLAM::ScalarType
    get_gaussian(mvSLAM::ScalarType mean = 0.0,
                 mvSLAM::ScalarType stddev = 1.0);

enum class RIG_TYPE
{
    CUBE,
    L_SHAPE,
    COUNT,
};

std::vector<mvSLAM::Vector3Type>
    get_rig_points(RIG_TYPE rig_type,
                   const mvSLAM::SO3 &rotation,
                   const mvSLAM::Vector3Type &translation,
                   mvSLAM::ScalarType scale = 1.0);
}
