#include <base/math.hpp>
#include "unit-test.hpp"
#include <iostream>
using namespace std;

using namespace unit_test;
static constexpr mvSLAM::ScalarType default_tolerance = 0.01;

template <typename MatrixType>
static bool
check_matrix_equal(const MatrixType &m1,
                   const MatrixType &m2,
                   mvSLAM::ScalarType tolerance)
{
    assert((m1.rows() == m2.rows()) && (m1.cols() == m2.cols()));
    for (auto i = m1.rows(); i > 0; --i)
        for (auto j = m1.cols(); j > 0; --j)
            ASSERT_EQUAL(m1(i-1, j-1), m2(i-1, j-1), tolerance);
    return true;
}

UNIT_TEST(SO3)
{
    const mvSLAM::Matrix3Type identity = mvSLAM::Matrix3Type::Identity();
    bool result;

    // default constructor
    mvSLAM::SO3 a;
    result = check_matrix_equal(a.get_matrix(),
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    // from Tait-Bryan angles
    mvSLAM::ScalarType roll = 0.1, pitch = -0.2, yaw = 0.3;
    mvSLAM::SO3 b(roll, pitch, yaw);
    ASSERT_EQUAL(roll, b.get_roll(), default_tolerance);
    ASSERT_EQUAL(pitch, b.get_pitch(), default_tolerance);
    ASSERT_EQUAL(yaw, b.get_yaw(), default_tolerance);

    // inverse
    result = check_matrix_equal(a.inverse().get_matrix(),
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    mvSLAM::Matrix3Type tmp = b.inverse().get_matrix() * b.get_matrix();
    result = check_matrix_equal(tmp,
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    // log and exp
    mvSLAM::Vector3Type so3 = a.ln();
    mvSLAM::Vector3Type zero = mvSLAM::Vector3Type::Zero();
    result = check_matrix_equal(so3, zero, default_tolerance);
    ASSERT_TRUE(result);
    result = check_matrix_equal(mvSLAM::SO3::exp(so3).get_matrix(),
                       a.get_matrix(),
                       default_tolerance);
    ASSERT_TRUE(result);
    so3 = b.ln();
    result = check_matrix_equal(mvSLAM::SO3::exp(so3).get_matrix(),
                       b.get_matrix(),
                       default_tolerance);
    ASSERT_TRUE(result);

    PASS();
}

UNIT_TEST(SE3)
{
    const mvSLAM::Matrix4Type identity = mvSLAM::Matrix4Type::Identity();
    bool result;

    // default constructor
    mvSLAM::SE3 a;
    result = check_matrix_equal(a.get_matrix(),
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    // from SO3 and translation
    mvSLAM::ScalarType roll = 0.1, pitch = -0.2, yaw = 0.3;
    mvSLAM::SO3 R(roll, pitch, yaw);
    mvSLAM::Vector3Type t;
    t << 1.0, 2.0, -3.0;
    mvSLAM::SE3 b(R, t);
    result = check_matrix_equal(b.rotation().get_matrix(),
                       R.get_matrix(),
                       default_tolerance);
    ASSERT_TRUE(result);

    result = check_matrix_equal(b.translation(),
                       t,
                       default_tolerance);
    ASSERT_TRUE(result);

    // inverse
    result = check_matrix_equal(a.inverse().get_matrix(),
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    mvSLAM::Matrix4Type tmp = b.inverse().get_matrix() * b.get_matrix();
    result = check_matrix_equal(tmp,
                       identity,
                       default_tolerance);
    ASSERT_TRUE(result);

    // log and exp
    mvSLAM::Vector6Type se3 = a.ln();
    mvSLAM::Vector6Type zero = mvSLAM::Vector6Type::Zero();
    result = check_matrix_equal(se3, zero, default_tolerance);
    ASSERT_TRUE(result);

    result = check_matrix_equal(mvSLAM::SE3::exp(se3).get_matrix(),
                       a.get_matrix(),
                       default_tolerance);
    ASSERT_TRUE(result);

    se3 = b.ln();
    result = check_matrix_equal(mvSLAM::SE3::exp(se3).rotation().get_matrix(),
                                R.get_matrix(),
                                default_tolerance);
    ASSERT_TRUE(result);
    result = check_matrix_equal(mvSLAM::SE3::exp(se3).get_matrix(),
                       b.get_matrix(),
                       default_tolerance);
    ASSERT_TRUE(result);
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}

