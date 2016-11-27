#pragma once
#include <system-config.hpp>
#include <opencv2/opencv.hpp>
#include <base/math.hpp>

namespace mvSLAM
{

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

Matrix3Type Mat_to_Matrix3Type(const cv::Mat &m);
cv::Mat Matrix3Type_to_Mat(const Matrix3Type &m);

Vector3Type Mat_to_Vector3Type(const cv::Mat &v);
cv::Mat Vector3Type_to_Mat(const Vector3Type &v);

}
