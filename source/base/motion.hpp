#pragma once
#include <base/math.hpp>

namespace mvSLAM
{

class MotionEstimate
{
public:
    const SE3 &mu()
    {
        return _mu;
    }

    SE3 &mu()
    {
        return _mu;
    }

    const Matrix6Type &covar()
    {
        return _covar;
    }

    Matrix6Type &covar()
    {
        return _covar;
    }

    const Matrix6Type &info()
    {
        return _covar.inverse();
    }
    
private:
    SE3 _mu;
    Matrix6Type _covar;
};

using Pose = SE3;
using PoseEstimate = MotionEstimate;
}
