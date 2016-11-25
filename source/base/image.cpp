#include <base/image.hpp>

namespace mvSLAM
{

bool operator==(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return (kp1.hash() == kp2.hash());
}

bool operator!=(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2)
{
    return !operator==(kp1, kp2);
}

template <typename EigenMatrixType>
static
EigenMatrixType Mat_to_Matrix(const cv::Mat &m)
{
    EigenMatrixType result;
    for (int i = 0; i < result.rows(); ++i)
        for (int j = 0; j < result.cols(); ++j)
            result(i, j) = m.at<ScalarType>(i, j);
    return result;
}

template <typename EigenMatrixType>
static
cv::Mat Matrix_to_Mat(const EigenMatrixType &m)
{
    cv::Mat result(m.rows(), m.cols(), CV_32FC1);
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < m.cols(); ++j)
            result.at<ScalarType>(i, j) = m(i, j);
    return result;
}

Matrix3Type Mat_to_Matrix3Type(const cv::Mat &m)
{
    return Mat_to_Matrix<Matrix3Type>(m);
}

cv::Mat Matrix3Type_to_Mat(const Matrix3Type &m)
{
    return Matrix_to_Mat<Matrix3Type>(m);
}

Vector3Type Mat_to_Vector3Type(const cv::Mat &v)
{
    return Mat_to_Matrix<Vector3Type>(v);
}

cv::Mat Vector3Type_to_Mat(const Vector3Type &v)
{
    return Matrix_to_Mat<Vector3Type>(v);
}

}
