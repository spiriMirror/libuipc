#pragma once
#include <type_define.h>


namespace uipc::backend::cuda
{
template <typename T, int M, int N>
UIPC_GENERIC Eigen::Matrix<T, M, N> lerp(const Eigen::Matrix<T, M, N>& src,
                                         const Eigen::Matrix<T, M, N>& dst,
                                         T                             alpha)
{
    alpha = std::clamp(alpha, T{0}, T{1});
    return src * (T{1} - alpha) + dst * alpha;
}

template <typename T>
UIPC_GENERIC T lerp(const T& src, const T& dst, T alpha)
{
    alpha = std::clamp(alpha, T{0}, T{1});
    return src * (T{1} - alpha) + dst * alpha;
}
}  // namespace uipc::backend::cuda
