#include <cassert>
#include <iostream>
#include <vision/estimator-RANSAC.hpp>
#include <vision/fundamental-matrix.hpp>
namespace mvSLAM
{

FundamentalMatrixEstimatorRANSAC::FundamentalMatrixEstimatorRANSAC(
        ScalarType max_error_sq_, size_t max_iteration_):
    max_error_sq(max_error_sq_), max_iteration(max_iteration_)
{
    assert(max_error_sq > epsilon);
    assert(max_iteration > 0);
}

bool
FundamentalMatrixEstimatorRANSAC::compute(const std::vector<Vector3Type> &p1,
                                          const std::vector<Vector3Type> &p2,
                                          Matrix3Type &F21,
                                          std::vector<uint8_t> &inlier_mask)
{
    assert(p1.size() == p2.size());

    const size_t pair_count = p1.size();
    if (pair_count < MIN_DATA_POINT_COUNT) // not enough points
    {
        std::cerr<<"Not enough points"<<std::endl;
        return false;
    }
    std::vector<size_t> index;
    index.reserve(pair_count);
    for (size_t i = 0; i < pair_count; ++i)
        index.push_back(i);
    
    ScalarType residual_best = infinity;
    size_t inlier_count_best = 0;
    
    for (size_t i = 0; i < max_iteration; ++i)
    {
        // draw a new sample
        // FIXME: random shuffle is disabled for debugging
        //std::shuffle(index.begin(), index.end(), std::default_random_engine());
        std::vector<Vector3Type> p1_sample, p2_sample;
        for (size_t j = 0; j < MIN_DATA_POINT_COUNT; ++j)
        {
            p1_sample.push_back(p1[index[j]]);
            p2_sample.push_back(p2[index[j]]);
        }
#if DEBUG_RANSAC 
        std::cout<<"sampled indexes:";
        for (size_t j = 0; j < MIN_DATA_POINT_COUNT; ++j)
            std::cout<<" "<<index[j];
        std::cout<<std::endl;
#endif

        // make a proposal based on the subset
        Matrix3Type F21_proposal;
        if (!propose_model(p1_sample, p2_sample, F21_proposal))
        {
            std::cerr<<"Cannot propose a model based on the sample."<<std::endl;
            continue;
        }

        // count inliers
        ScalarType residual_proposal;
        size_t inlier_count_proposal;
        std::vector<uint8_t> inlier_mask_proposal;
        inlier_count_proposal = count_inliers(p1,
                                              p2,
                                              F21_proposal,
                                              inlier_mask_proposal,
                                              residual_proposal);
        std::cout<<inlier_count_proposal<<" inliers, residual = "<<residual_proposal<<std::endl;

        // update best solution
        if ((inlier_count_proposal > inlier_count_best) ||
            ((inlier_count_proposal == inlier_count_best) &&
             (residual_proposal < residual_best)))
        {
            inlier_count_best = inlier_count_proposal;
            residual_best = residual_proposal;
            std::swap(F21, F21_proposal);
            std::swap(inlier_mask, inlier_mask_proposal);
        }
    }
    std::cout<<"inlier_count_best = "<<inlier_count_best<<std::endl;
    std::cout<<"residual_best = "<<residual_best<<std::endl;

    return (inlier_count_best > 0);
}

bool
FundamentalMatrixEstimatorRANSAC::propose_model(const std::vector<Vector3Type> &p1_sample,
                                                const std::vector<Vector3Type> &p2_sample,
                                                Matrix3Type &F21)
{
    return find_fundamental_matrix(p1_sample, p2_sample, F21);
}

size_t 
FundamentalMatrixEstimatorRANSAC::count_inliers(const std::vector<Vector3Type> &p1,
                                                const std::vector<Vector3Type> &p2,
                                                const Matrix3Type &F21,
                                                std::vector<uint8_t> &inlier_mask,
                                                ScalarType &residual)
{
    const size_t pair_count = p1.size();
    size_t inlier_count = 0;
    residual = static_cast<ScalarType>(0);
    inlier_mask.clear();
    inlier_mask.reserve(pair_count);
    for (size_t i = 0; i < pair_count; ++i)
    {
        ScalarType r = p2[i].transpose() * F21 * p1[i];
        if (r < static_cast<ScalarType>(0))
            r = -r;
        if (r < max_error_sq)
        {
            ++inlier_count;
            residual += r;
            inlier_mask.push_back(1);
        }
        else
        {
            inlier_mask.push_back(0);
        }
    }
    return inlier_count;
}

}
