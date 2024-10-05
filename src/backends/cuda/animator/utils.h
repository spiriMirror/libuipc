#pragma once
#include <type_define.h>


namespace uipc::backend::cuda
{
template <typename T, int M, int N>
UIPC_GENERIC Eigen::Matrix<T, M, N> lerp(const Eigen::Matrix<T, M, N>& a,
                                         const Eigen::Matrix<T, M, N>& b,
                                         T                             alpha)
{
    alpha = std::clamp(alpha, T{0}, T{1});
    return a * alpha + b * (T{1} - alpha);
}

template <typename T>
UIPC_GENERIC T lerp(const T& a, const T& b, T alpha)
{
    alpha = std::clamp(alpha, T{0}, T{1});
    return a * alpha + b * (T{1} - alpha);
}
}  // namespace uipc::backend::cuda
