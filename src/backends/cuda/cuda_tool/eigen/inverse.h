#pragma once
#include <cuda_tool/eigen/gauss_elimination.h>
#include <cuda_tool/eigen/analytic_inverse.h>

namespace uipc::backend::cuda_tool::eigen
{
template <typename T, int N, typename InverseAlgorithm = cuda_tool::eigen::GaussEliminationInverse>
UIPC_INLINE UIPC_GENERIC Eigen::Matrix<T, N, N> inverse(const Eigen::Matrix<T, N, N>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = cuda_tool::eigen::AnalyticalInverse>
UIPC_INLINE UIPC_GENERIC Eigen::Matrix<T, 2, 2> inverse(const Eigen::Matrix<T, 2, 2>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = cuda_tool::eigen::AnalyticalInverse>
UIPC_INLINE UIPC_GENERIC Eigen::Matrix<T, 3, 3> inverse(const Eigen::Matrix<T, 3, 3>& m)
{
    return InverseAlgorithm{}(m);
}

template <typename T, typename InverseAlgorithm = cuda_tool::eigen::AnalyticalInverse>
UIPC_INLINE UIPC_GENERIC Eigen::Matrix<T, 4, 4> inverse(const Eigen::Matrix<T, 4, 4>& m)
{
    return InverseAlgorithm{}(m);
}
}  // namespace uipc::backend::cuda_tool::eigen
