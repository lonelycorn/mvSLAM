#pragma once
#include <base/math.hpp>
#include <vector>
namespace mvSLAM
{
/** Compute fundamental matrix from the give point pairs.
 * Currently using 8-Point method.
 * @param [in] p1_sample    points from image 1, in homogeneous representation (x, y, 1)
 *                          must be aligned with @p p2_sample
 * @param [in] p2_sample    points from image 2, in homogeneous representation (x, y, 1)
 *                          must be aligned with @p p1_sample
 * @param [out] F21         the computed fundamental matrix, i.e. p2.T * F21 * p1 == 0.
 * @return whether the computation was successful.
 */
bool
find_fundamental_matrix(const std::vector<Vector3Type> &p1_sample,
                        const std::vector<Vector3Type> &p2_sample,
                        Matrix3Type &F21);
}


