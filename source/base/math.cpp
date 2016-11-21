#include <base/math.hpp>

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
        A = 1.0 - sqr(theta) / 6.0;
        B = -0.5 + sqr(theta) / 24.0;
    }
    else // regular formula
    {
        A = std::sin(theta) / theta;
        B = (1.0 - std::cos(theta)) / sqr(theta);
    }
    const Matrix3Type K = skew_symmetric_matrix(v); // NOTE: v is not normalized
    return Matrix3Type::Identity() + A * K + B * K * K;
}

}
