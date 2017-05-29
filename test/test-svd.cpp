#include <Eigen/Core>
#include <math/matrix.hpp>
#include <math/svd.hpp>
#include "unit-test.hpp"

using namespace unit_test;

constexpr mvSLAM::ScalarType tolerance = 0.001;

UNIT_TEST(svd_identity_3x4)
{
    using MatrixType = Eigen::Matrix<mvSLAM::ScalarType, 3, 3>;
    MatrixType A = MatrixType::Identity();
    mvSLAM::SVD<MatrixType> svd(A);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    auto s = svd.singularValues();

    for (int i = 0; i < 3; ++i)
    {
        ASSERT_EQUAL(s[i], 1.0, tolerance);
        for (int j = 0; j < 3; ++j)
        {
            // both U and V are identity matrices
            ASSERT_EQUAL(U(i, j), A(i, j), tolerance);
            ASSERT_EQUAL(V(i, j), A(i, j), tolerance);
        }

    }
    PASS();
}

UNIT_TEST(svd_homogeneous_2x3)
{
    using MatrixType = Eigen::Matrix<mvSLAM::ScalarType, 2, 3>;
    MatrixType A;
    A << 1, 1, 0,
         0, 1, 1;
    mvSLAM::SVD<MatrixType> svd(A);
    Eigen::Matrix<mvSLAM::ScalarType, 3, 1> X = svd.matrixV().col(2);

    ASSERT_EQUAL(X[0], 0.57735, tolerance);
    ASSERT_EQUAL(X[1],-0.57735, tolerance);
    ASSERT_EQUAL(X[2], 0.57735, tolerance);

    Eigen::Matrix<mvSLAM::ScalarType, 2, 1>  b = A * X;
    for (int i = 0; i < 2; ++i)
        ASSERT_EQUAL(b[i], 0, tolerance);
    PASS();
}

UNIT_TEST(svd_random_4x5)
{
    using MatrixType = Eigen::Matrix<mvSLAM::ScalarType, 4, 5>;
    MatrixType A(MatrixType::Random());
    mvSLAM::SVD<MatrixType> svd(A);
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    auto s = svd.singularValues();
    MatrixType S(MatrixType::Identity());
    for (int i = 0; i < 4; ++i)
        S(i, i) = s[i];
    MatrixType B = U * S * V.transpose();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 5; ++j)
            ASSERT_EQUAL(B(i, j), A(i, j), tolerance);
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
};
