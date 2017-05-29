#include <math/lie-group.hpp>

namespace mvSLAM
{
Matrix3Type
skew_symmetric_matrix(const Vector3Type &v)
{
    Matrix3Type m;
    m <<    0, -v(2),  v(1),
         v(2),     0, -v(0),
        -v(1),  v(0),     0;
    return m;
}

Matrix3Type rodrigues(const Vector3Type &v)
{
    ScalarType theta = v.norm();
    ScalarType A; // sin(theta) / theta
    ScalarType B; // (1 - cos(theta)) / sqr(theta)
    if (theta < epsilon) // Taylor expansion
    {
        A = 1.0 - sqr(theta) / 6.0; // + pow(x, 4) / 120
        B = 0.5 - sqr(theta) / 24.0; // + pow(x, 4) / 720
    }
    else // regular formula
    {
        A = std::sin(theta) / theta;
        B = (1.0 - std::cos(theta)) / sqr(theta);
    }
    const Matrix3Type K = skew_symmetric_matrix(v); // NOTE: v is not normalized
    return Matrix3Type::Identity() + A * K + B * K * K;
}

Matrix3x4Type SE3_to_Matrix3x4Type(const SE3 &s)
{
    Matrix3x4Type m;
    m.block<3, 3>(0, 0) = s.rotation().get_matrix();
    m.block<3, 1>(0, 3) = s.translation();
    return m;
}

SE3 Matrix3x4Type_to_SE3(const Matrix3x4Type &m)
{
    Matrix3Type R = m.block<3, 3>(0, 0);
    Vector3Type t = m.block<3, 1>(0, 3);
    SE3 s(SO3(R), t);
    return s;
}

std::ostream &operator<<(std::ostream &out, const SO3 &instance)
{
    out<<"[Rotation]\n"<<instance._R;
    return out;
}

std::ostream &operator<<(std::ostream &out, const SE3 &instance)
{
    out<<"[translation]\n"<<instance._t;
    out<<std::endl; // delimeter
    out<<instance._R;
    return out;
}

}
