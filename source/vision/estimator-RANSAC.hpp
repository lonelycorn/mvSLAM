#pragma once
#include <system-config.hpp>
#include <math/matrix.hpp>
#include <cstdint>
#include <vector>

namespace mvSLAM
{
// TODO: make a more generic EstimatorRANSAC
class FundamentalMatrixEstimatorRANSAC
{
public:
    FundamentalMatrixEstimatorRANSAC(ScalarType max_error_sq,
                                     size_t max_iteration);

    /**
     * @brief Compute the fundamental matrix F21 such that p2.T * F21 * p1 == 0.
     * @return whether the computation was successful.
     */
    bool
    compute(const std::vector<Vector3Type> &p1,
            const std::vector<Vector3Type> &p2,
            Matrix3Type &F21,
            std::vector<uint8_t> &inlier_mask);

private:
    /**
     * @brief Propose a model using the provided samples.
     * The sample should contain the minimal number of data points to
     * generate a model.
     */
    bool
    propose_model(const std::vector<Vector3Type> &p1_sample,
                  const std::vector<Vector3Type> &p2_sample,
                  Matrix3Type &F21);
    
    /**
     * @brief Count the number of inliers using the given model.
     */
    size_t 
    count_inliers(const std::vector<Vector3Type> &p1,
                  const std::vector<Vector3Type> &p2,
                  const Matrix3Type &F21,
                  std::vector<uint8_t> &inlier_mask,
                  ScalarType &residual);
    
    const ScalarType max_error_sq; // max_error_sq = |x2.T * F21 * x1|
    const size_t max_iteration;
    static constexpr size_t MIN_DATA_POINT_COUNT = 8;
};

}
