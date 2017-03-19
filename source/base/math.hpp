#pragma once
#include <system-config.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

namespace mvSLAM
{

using Matrix2Type = Eigen::Matrix<ScalarType, 2, 2>;
using Vector2Type = Eigen::Matrix<ScalarType, 2, 1>;

using Matrix3Type = Eigen::Matrix<ScalarType, 3, 3>;
using Vector3Type = Eigen::Matrix<ScalarType, 3, 1>;

using Matrix6Type = Eigen::Matrix<ScalarType, 6, 6>;
using Vector6Type = Eigen::Matrix<ScalarType, 6, 1>;

using Matrix4Type = Eigen::Matrix<ScalarType, 4, 4>;
using Matrix3x4Type = Eigen::Matrix<ScalarType, 3, 4>;
using Vector4Type = Eigen::Matrix<ScalarType, 4, 1>;

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

/** get the skew symmetric matrix for the given vector.
 *  Example:
 *      a x b == skew_symmetric_matrix(a) b
 */
Matrix3Type
skew_symmetric_matrix(const Vector3Type &v);

/** Rodrigues' rotation formula.
 * see https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
 * @param [in] v    axis-angle representation of the rotation.
 * @return a rotation matrix equivalent to the axis-angle representation.
 */
Matrix3Type
rodrigues(const Vector3Type &v);

class SO3
{
public:
    /** Default ctor.
     */
    SO3(): _R(Matrix3Type::Identity()) {}

    explicit SO3(const Matrix3Type &m): _R(m)
    {
        // FIXME: we may need more checks here
        assert(_R.norm() > epsilon);
        rectify();
    }

    /** Construct using Tait-Bryan angles.
     *  Sequence of rotation: z-y-x'', i.e. yaw, pitch then roll
     */
    SO3(ScalarType roll, ScalarType pitch, ScalarType yaw): _R()
    {
        Matrix3Type Rx;
        Rx << 1.0, 0.0, 0.0,
              0.0, std::cos(roll), -std::sin(roll),
              0.0, std::sin(roll),  std::cos(roll);
        Matrix3Type Ry;
        Ry << std::cos(pitch), 0.0, std::sin(pitch),
              0.0, 1.0, 0.0,
              -std::sin(pitch), 0.0, std::cos(pitch);
        Matrix3Type Rz;
        Rz << std::cos(yaw), -std::sin(yaw), 0.0,
              std::sin(yaw),  std::cos(yaw), 0.0,
              0.0, 0.0, 1.0;
        _R = Rz * Ry * Rx;
    }

    /** Construct using so3.
     */
    SO3(const Vector3Type &so3): _R(rodrigues(so3)) {}

    ~SO3() {}

    SO3(const SO3 &) = default;
    SO3(SO3 &&) = default;
    SO3 &operator=(const SO3 &) = default;
    SO3 &operator=(SO3 &&) = default;

    const Matrix3Type &get_matrix() const
    {
        return _R;
    }

    SO3 inverse() const
    {
        Matrix3Type R = _R.transpose();
        return SO3(R);
    }

    /** Make sure this is a valid SO3, i.e. orthonormal.
     * see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
     */
    void rectify()
    {
        Vector3Type u0 = _R.row(0);
        u0 /= u0.norm();

        Vector3Type u1 = _R.row(1);
        u1 = (u1 - u1.dot(u0) * u0);

        Vector3Type u2 = u0.cross(u1);
        _R.row(0) = u0;
        _R.row(1) = u1;
        _R.row(2) = u2;
    }

    ScalarType get_roll() const
    {
        return std::atan2(_R(2, 1), _R(2, 2));
    }

    ScalarType get_pitch() const
    {
        return std::asin(-_R(2, 0));
    }

    ScalarType get_yaw() const
    {
        return std::atan2(_R(1, 0), _R(0, 0));
    }

    Vector3Type operator*(const Vector3Type &v) const
    {
        return _R * v;
    }

    SO3 operator*(const SO3 &rhs) const
    {
        Matrix3Type R = _R * rhs._R;
        return SO3(R);
    }

    Matrix3Type adjoint() const
    {
        return _R;
    }

    void reset()
    {
        _R = Matrix3Type::Identity();
    }

    /** Get the angle-axis representation of the SO3.
     *  This is the inverse operation of exp().
     *  see https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
     */
    Vector3Type ln() const
    {
        const ScalarType cos_theta = constrain(0.5 * (_R.trace() - 1.0), -1.0, 1.0);
        const ScalarType theta = std::acos(cos_theta);

        // if v is the axis, R v == v must hold
        // one solution would be (R - R.T()) == skew_symmetric_matrix(v)
        // NOTE: v.norm() == 2 * sin(theta)
        Vector3Type v;
        v << _R(2, 1) - _R(1, 2),
             _R(0, 2) - _R(2, 0),
             _R(1, 0) - _R(0, 1);

        ScalarType A; // theta / sin(theta) / 2
        if (theta < taylor_threshold) // Taylor expansion. sin(theta) < theta
        {
            A = (1.0 + sqr(theta) / 6.0) * 0.5;
        }
        else // regular formula
        {
            A = 0.5 * theta / std::sin(theta);
        }
        v *= A;
        return v;
    }

    /** Convert the angle-axis representation to SO3.
     *  This is the inverse operation of ln().
     *  see https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
     */
    static SO3 exp(const Vector3Type &so3)
    {
        return SO3(so3);
    }

    friend std::ostream &operator<<(std::ostream &out, const SO3 &instance);

private:
    Matrix3Type _R;
};


class SE3
{
public:
    /// Construct using given rotation and translation
    SE3(const SO3 &r, const Vector3Type &t): _R(r), _t(t) {}
    /// Ctor
    SE3(): _R(), _t(Vector3Type::Zero()) {}
    ~SE3() {}

    SE3(const SE3 &) = default;
    SE3(SE3 &&) = default;
    SE3 &operator=(const SE3 &) = default;
    SE3 &operator=(SE3 &&) = default;

    const SO3 &rotation() const
    {
        return _R;
    }

    const Vector3Type &translation() const
    {
        return _t;
    }

    Matrix4Type get_matrix() const
    {
        Matrix4Type result = Matrix4Type::Zero();
        result.block<3, 3>(0, 0) = _R.get_matrix();
        result.block<3, 1>(0, 3) = _t;
        result(3, 3) = static_cast<ScalarType>(1);
        return result;
    }

    SE3 inverse() const
    {
        return SE3(_R.inverse(), -(_R.get_matrix().transpose() * _t));
    }

    void reset()
    {
        _R.reset();
        _t = Vector3Type::Zero();
    }

    Vector3Type operator*(const Vector3Type &v) const
    {
        return _R * v + _t;
    }

    SE3 operator*(const SE3 &rhs) const
    {
        SO3 R = _R * rhs._R;
        Vector3Type t = _R * rhs._t + _t;
        return SE3(R, t);
    }

    /** Get the adjoint matrix for this transform.
     * The adjoint matrix is used to transform tangent vectors from
     * one pose to another pose
     */
    Matrix6Type adjoint() const
    {
        assert(false);
    }

    Vector6Type ln() const
    {
        const Vector3Type w = _R.ln();
        const ScalarType theta = w.norm();
        // ScalarType A; // sin(theta) / theta
        // ScalarType B; // (1 - cos(theta)) / (theta * theta)
        ScalarType G; // (1 - A / B / 2) / (theta * theta)
        if (theta < taylor_threshold) // Taylor expansion
        {
            G = 1.0 / 12.0 + sqr(theta) / 720.0;
        }
        else
        {
            ScalarType A = std::sin(theta) / theta;
            ScalarType B = (1.0 - std::cos(theta)) / sqr(theta);
            G = (1.0 - 0.5 * A / B) / sqr(theta);
        }
        const Matrix3Type K = skew_symmetric_matrix(w);
        const Matrix3Type V_inv = Matrix3Type::Identity() - 0.5 * K + G * K * K;
        const Vector3Type u = V_inv * _t;
        Vector6Type se3;
        se3.block<3, 1>(0, 0) = u;
        se3.block<3, 1>(3, 0) = w;
        return se3;
    }

    /** Create by exponential mapping.
     * see http://www.ethaneade.org/lie.pdf
     * @param [in] se3  representation in the tagent space. Translation part first.
     */
    static SE3 exp(const Vector6Type &se3)
    {
        const Vector3Type u = se3.block<3, 1>(0, 0); // translation part
        const Vector3Type w = se3.block<3, 1>(3, 0); // rotation part
        const ScalarType theta = w.norm();
        ScalarType A; // sin(theta) / theta
        ScalarType B; // (1 - cos(theta)) / (theta * theta)
        ScalarType C; // (1 - A) / (theta * theta)
        if (theta < taylor_threshold) // Taylor expansion
        {
            A = 1.0 - sqr(theta) / 6.0; // + pow(x, 4) / 120
            B = 0.5 - sqr(theta) / 24.0; // + pow(x, 4) / 720
            C = 1.0 / 6.0 - sqr(theta) / 120.0; // + pow(x, 4) / 5040
        }
        else
        {
            A = std::sin(theta) / theta;
            B = (1.0 - std::cos(theta)) / sqr(theta);
            C = (1.0 - A) / sqr(theta);
        }
        const SO3 R = SO3::exp(w);
        const Matrix3Type K = skew_symmetric_matrix(w);
        const Matrix3Type V = Matrix3Type::Identity() + B * K + C * K * K;
        return SE3(R, V * u);
    }

    friend std::ostream &operator<<(std::ostream &out, const SE3 &instance);

private:
    SO3 _R;
    Vector3Type _t;
};

std::ostream &operator<<(std::ostream &out, const SO3 &instance);

std::ostream &operator<<(std::ostream &out, const SE3 &instance);

SE3 Matrix3x4Type_to_SE3(const Matrix3x4Type &m);
Matrix3x4Type SE3_to_Matrix3x4Type(const SE3 &s);

}
