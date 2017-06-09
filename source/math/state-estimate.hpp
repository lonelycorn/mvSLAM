#pragma once
namespace mvSLAM
{
/// Random variables with Gaussian distribution.
template <typename MeanType, typename CovarType>
class StateEstimate 
{
public:
    const MeanType &mean() const
    {
        return _mean;
    }

    MeanType &mean()
    {
        return _mean;
    }

    const CovarType &covar() const
    {
        return _covar;
    }

    CovarType &covar()
    {
        return _covar;
    }

    const CovarType &info() const
    {
        // NOTE: we may want to cache this value
        return _covar.inverse();
    }

    /**
     * @brief Ctor.
     * @note we may want to use information matrix instead
     */
    StateEstimate(const MeanType &mean, const CovarType &covar):
        _mean(mean), _covar(covar)
    {
    }

    /// Default ctor.
    StateEstimate():
        _mean(), _covar(CovarType::Identity())
    {
    }
    ~StateEstimate()
    {
    }

private:
    MeanType _mean;
    CovarType _covar;
};

}
