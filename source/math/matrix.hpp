#pragma once
#include <system-config.hpp>
#include <Eigen/LU> // NOTE: must #include here in order for MatrixBase::inverse() to work
#include <Eigen/Geometry> // NOTE: must #include here in order for MatrixBase::cross() to work
#include <Eigen/Core>

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
}
