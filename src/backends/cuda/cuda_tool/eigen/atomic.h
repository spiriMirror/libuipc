#pragma once
#include <cuda_tool/atomic.h>

namespace uipc::backend::cuda_tool::eigen
{
template <typename T, int M, int N>
UIPC_GENERIC Eigen::Matrix<T, M, N> atomic_add(Eigen::Matrix<T, M, N>& dst,
                                               const Eigen::Matrix<T, M, N>& src)
{
    Eigen::Matrix<T, M, N> ret;

#pragma unroll
    for(int j = 0; j < N; ++j)
#pragma unroll
        for(int i = 0; i < M; ++i)
        {
            ret(i, j) = cuda_tool::atomic_add(&dst(i, j), src(i, j));
        }
    return ret;
}

template <typename T, int M, int N>
UIPC_GENERIC Eigen::Matrix<T, M, N> atomic_add(Eigen::Map<Eigen::Matrix<T, M, N>>& dst,
                                               const Eigen::Matrix<T, M, N>& src)
{
    Eigen::Matrix<T, M, N> ret;

#pragma unroll
    for(int j = 0; j < N; ++j)
#pragma unroll
        for(int i = 0; i < M; ++i)
        {
            ret(i, j) = cuda_tool::atomic_add(&dst(i, j), src(i, j));
        }

    return ret;
}
}  // namespace uipc::backend::cuda_tool::eigen