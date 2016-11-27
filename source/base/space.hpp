#pragma once
#include <base/math.hpp>

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

class Point3D
{
public:
    Point3D(const Vector3Type &v) : _coordinates(v) {}
    Point3D(): _coordinates() {}
    Point3D(ScalarType x, ScalarType y, ScalarType z) : _coordinates()
    {
        _coordinates << x, y, z;
    }
    ScalarType x() const { return _coordinates[0]; }
    ScalarType y() const { return _coordinates[1]; }
    ScalarType z() const { return _coordinates[2]; }
    Vector3Type get_vector() const { return _coordinates; }
    ScalarType operator()(size_t index) const { return _coordinates[index]; }
    ScalarType &operator()(size_t index) { return _coordinates[index]; }
private:
    Vector3Type _coordinates;
};
using Point3DUncertainty = Matrix3Type;
using Point3DEstimate = StateEstimate<Point3D, Point3DUncertainty>;

}
