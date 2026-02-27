#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <muda/logger/logger_viewer.h>

namespace muda
{
template <typename T, int M, int N>
MUDA_DEVICE LogProxy operator<<(LogProxy o, const Eigen::Matrix<T, M, N>& val);

template <typename T, int M, int N, int MapOptions, typename StrideType>
MUDA_DEVICE LogProxy operator<<(
    LogProxy o, const Eigen::Map<Eigen::Matrix<T, M, N>, MapOptions, StrideType>& val);

template <typename T, int M, int N, int MapOptions, typename StrideType>
MUDA_DEVICE LogProxy operator<<(
    LogProxy o,
    const Eigen::Map<const Eigen::Matrix<T, M, N>, MapOptions, StrideType>& val);

template <typename T>
MUDA_DEVICE LogProxy operator<<(LogProxy o, const Eigen::MatrixX<T>& val);

template <typename T>
MUDA_DEVICE LogProxy operator<<(LogProxy o, const Eigen::VectorX<T>& val);

template <typename T>
MUDA_DEVICE LogProxy operator<<(LogProxy o, const Eigen::RowVectorX<T>& val);
}  // namespace muda

#include "details/log_proxy.inl"