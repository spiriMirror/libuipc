#pragma once
#include <numeric>

namespace muda
{
class LinearSystemSolveTolerance
{
    double m_solve_sparse_error_threshold = -1.0;

  public:
    void solve_sparse_error_threshold(double threshold)
    {
        m_solve_sparse_error_threshold = threshold;
    }

    template <typename T>
    T solve_sparse_error_threshold()
    {
        if(m_solve_sparse_error_threshold < 0.0)
        {
            constexpr auto eps = std::numeric_limits<T>::epsilon();
            return eps;
        }
        else
        {
            return static_cast<T>(m_solve_sparse_error_threshold);
        }
    }
};
}  // namespace muda