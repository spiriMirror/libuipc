#pragma once
#include <Eigen/Core>
#include <uipc/common/span.h>

namespace uipc
{
template <std::floating_point T>
inline auto as_eigen(span<T> s)
{
    return Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>(s.data(), s.size());
}

template <std::floating_point T>
inline auto as_eigen(span<const T> s)
{
    return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>(s.data(), s.size());
}
}  // namespace uipc
