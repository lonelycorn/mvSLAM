#pragma once
#include <system-config.hpp>
#include <math/lie-group.hpp>
#include <vector>

namespace unit_test
{

mvSLAM::ScalarType
    get_gaussian(mvSLAM::ScalarType mean = 0.0,
                 mvSLAM::ScalarType stddev = 1.0);

// draw a sample from a zero-mean isotropic Gaussian distribution on SE3
mvSLAM::SE3
    get_SE3_sample(mvSLAM::ScalarType isotropic_noise_stddev);

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

bool check_similar_SE3(const mvSLAM::SE3 &T1,
                       const mvSLAM::SE3 &T2,
                       mvSLAM::ScalarType tol);
}
