#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <muda/ext/eigen/inverse/gauss_elimination.h>
#include <muda/ext/eigen/inverse/analytic_inverse.h>

namespace muda::eigen
{
template <typename T, int N, typename InverseAlgorithm = muda::eigen::GaussEliminationInverse>
MUDA_INLINE MUDA_GENERIC Eigen::Matrix<T, N, N> inverse(const Eigen::Matrix<T, N, N>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = muda::eigen::AnalyticalInverse>
MUDA_INLINE MUDA_GENERIC Eigen::Matrix<T, 2, 2> inverse(const Eigen::Matrix<T, 2, 2>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = muda::eigen::AnalyticalInverse>
MUDA_INLINE MUDA_GENERIC Eigen::Matrix<T, 3, 3> inverse(const Eigen::Matrix<T, 3, 3>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = muda::eigen::AnalyticalInverse>
MUDA_INLINE MUDA_GENERIC Eigen::Matrix<T, 4, 4> inverse(const Eigen::Matrix<T, 4, 4>& m)
{
    return InverseAlgorithm{}(m);
}
}  // namespace muda::eigen
