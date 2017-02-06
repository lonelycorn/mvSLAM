#pragma once
#include <system-config.hpp>
#include <Eigen/Core>
#include <base/opencv.hpp>
#include <base/debug.hpp>

namespace mvSLAM
{
#ifdef USE_OPENCV_SVD
/** Wrap cv::SVDecomp with an interface (kinda) compatible with
 * Eigen::JacobiSVD
 */
template <typename EigenMatrixType>
class SVD
{
public:
    // matrix traits
    static constexpr int ROWS = EigenMatrixType::RowsAtCompileTime;
    static constexpr int COLS = EigenMatrixType::ColsAtCompileTime;
    static constexpr int DIAG_SIZE = constexpr_min(ROWS, COLS);

    using MatrixUType = Eigen::Matrix<ScalarType, ROWS, ROWS>;
    using MatrixVType = Eigen::Matrix<ScalarType, COLS, COLS>;
    using SingularValuesType = Eigen::Matrix<ScalarType, DIAG_SIZE, 1>;

    SVD(const EigenMatrixType &matrix,
        unsigned int computation_option = 0):
        m_U(), m_V(), m_D()
    {
        compute(matrix, computation_option);
    }

    ~SVD() = default;
    SVD(const SVD &) = delete;
    SVD(SVD &&) = delete;
    SVD &operator=(const SVD &) = delete;
    SVD &operator=(SVD &&) = delete;

    const MatrixUType &matrixU() const
    {
        return m_U;
    }

    const MatrixVType &matrixV() const
    {
        return m_V;
    }

    const SingularValuesType &singularValues() const
    {
        return m_D;
    }

private:
    MatrixUType m_U;
    MatrixVType m_V;
    SingularValuesType m_D;

    void compute(const EigenMatrixType &matrix,
                 unsigned int computation_option)
    {
        (void) computation_option; // ignored; using the same options
        cv::Mat A = Matrix_to_Mat<EigenMatrixType>(matrix);
        cv::Mat u, w, vt;
        cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        m_U = Mat_to_Matrix<MatrixUType>(u);
        m_V = Mat_to_Matrix<MatrixVType>(vt).transpose();
        for (int i = 0; i < DIAG_SIZE; ++i)
        {
            m_D[i] = w.at<ScalarType>(i);
        }
    }
};
#else // USE_OPENCV_SVD
using SVD = Eigen::JacobiSVD;
#endif
}
