#pragma once
#include <base/math.hpp>
#include <iostream>

namespace mvSLAM
{

template <typename MuType, typename CovarType>
class StateEstimate 
{
public:
    const MuType &mu() const
    {
        return _mu;
    }

    MuType &mu()
    {
        return _mu;
    }

    const CovarType &covar() const
    {
        return _covar;
    }

    CovarType &covar()
    {
        return _covar;
    }

    const CovarType &info() const
    {
        return _covar.inverse();
    }
    /**
     * @brief Ctor.
     * @note we may want to use information matrix instead
     */
    StateEstimate(const MuType &mu, const CovarType &covar):
        _mu(mu), _covar(covar)
    {
    }
    /// Default ctor.
    StateEstimate():
        _mu(), _covar(CovarType::Identity())
    {
    }
    ~StateEstimate() {}
private:
    MuType _mu;
    CovarType _covar;
};

using Pose = SE3;
using PoseUncertainty = Matrix6Type;
using PoseEstimate = StateEstimate<Pose, PoseUncertainty>;

using Point3D = Vector3Type;
using Point3DUncertainty = Matrix3Type;
using Point3DEstimate = StateEstimate<Point3D, Point3DUncertainty>;

}
