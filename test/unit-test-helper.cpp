#include "unit-test-helper.hpp"
#include <cassert>

namespace unit_test
{
mvSLAM::ScalarType
get_gaussian(mvSLAM::ScalarType mean, mvSLAM::ScalarType stddev)
{
    static std::random_device rd;
    static std::mt19937 generator(rd());
    static std::normal_distribution<mvSLAM::ScalarType> standard_gaussian;
    return standard_gaussian(generator) * stddev + mean;
}

mvSLAM::SE3
get_SE3_sample(mvSLAM::ScalarType isotropic_noise_stddev)
{
    assert(isotropic_noise_stddev > static_cast<mvSLAM::ScalarType>(0));

    mvSLAM::Vector6Type delta;
    for (int i = 0; i < 6; ++i)
    {
        delta[i] = get_gaussian(0.0, isotropic_noise_stddev);
    }
    return mvSLAM::SE3::exp(delta);

}

static void
transform_points(std::vector<mvSLAM::Vector3Type> &points,
                 const mvSLAM::SO3 &rotation,
                 const mvSLAM::Vector3Type &translation,
                 mvSLAM::ScalarType scale)
{
    for (auto &p : points)
    {
        p =  rotation * (scale * p) + translation;
    }
}


std::vector<mvSLAM::Vector3Type>
get_rig_points(RIG_TYPE rig_type,
               const mvSLAM::SO3 &rotation,
               const mvSLAM::Vector3Type &translation,
               mvSLAM::ScalarType scale)
{
    std::vector<mvSLAM::Vector3Type> points;
    switch (rig_type)
    {
    case RIG_TYPE::CUBE:
        points.reserve(8);
        points.emplace_back(-1, -1, -1);
        points.emplace_back(-1, -1, +1);
        points.emplace_back(-1, +1, -1);
        points.emplace_back(-1, +1, +1);
        points.emplace_back(+1, -1, -1);
        points.emplace_back(+1, -1, +1);
        points.emplace_back(+1, +1, -1);
        points.emplace_back(+1, +1, +1);
        break;
    case RIG_TYPE::L_SHAPE:
        points.reserve(8);
        points.emplace_back(1, 0, 0);
        points.emplace_back(0, 0, 0);
        points.emplace_back(0, 2, 0);
        points.emplace_back(1, 0, 3);
        points.emplace_back(0, 0, 3);
        points.emplace_back(0, 2, 3);
        points.emplace_back(0.5, 0.0, 1.5);
        points.emplace_back(0.0, 1.0, 1.5);
        break;
    default:
        assert("Invalid RIG_TYPE.");
    }

    transform_points(points, rotation, translation, scale);
    return points;
}

bool check_similar_SE3(const mvSLAM::SE3 &T1,
                       const mvSLAM::SE3 &T2,
                       mvSLAM::ScalarType tol)
{
    auto t1 = T1.ln();
    auto t2 = T2.ln();
    for (size_t i = 0; i < 6; ++i)
    {
        if ((t1[i] < t2[i] - tol) ||
            (t1[i] > t2[i] + tol))
        {
            return false;
        }
    }
    return true;
}

}
