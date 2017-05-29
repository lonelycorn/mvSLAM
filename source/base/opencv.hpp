#pragma once
#include <system-config.hpp>
#include <math/matrix.hpp>
#include <opencv2/opencv.hpp>

/**
 * Helper functions to interface with OpenCV
 */
namespace mvSLAM
{

/** cv::Mat traits
 */
template <typename T>
struct cv_Mat_traits
{
};

template <>
struct cv_Mat_traits<double>
{
    static constexpr int DataType = CV_64FC1;
};

template <>
struct cv_Mat_traits<float>
{
    static constexpr int DataType = CV_32FC1;
};

/** Conversion from cv::Mat
 */
template <typename EigenMatrixType>
EigenMatrixType Mat_to_Matrix(const cv::Mat &m)
{
    EigenMatrixType result;
    for (int i = 0; i < result.rows(); ++i)
        for (int j = 0; j < result.cols(); ++j)
            result(i, j) = m.at<ScalarType>(i, j);
    return result;
}

/** Conversion to cv::Mat
 */
template <typename EigenMatrixType>
cv::Mat Matrix_to_Mat(const EigenMatrixType &m)
{
    cv::Mat result(m.rows(), m.cols(), cv_Mat_traits<ScalarType>::DataType);
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < m.cols(); ++j)
            result.at<ScalarType>(i, j) = m(i, j);
    return result;
}
}
