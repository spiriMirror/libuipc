#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::distance
{
template <typename T>
MUDA_GENERIC void point_point_distance2(const Eigen::Vector<T, 3>& a,
                                       const Eigen::Vector<T, 3>& b,
                                       T&                         dist2);

template <typename T>
MUDA_GENERIC void point_point_distance2_gradient(const Eigen::Vector<T, 3>& a,
                                                const Eigen::Vector<T, 3>& b,
                                                Eigen::Vector<T, 6>& grad);

template <typename T>
MUDA_GENERIC void point_point_distance2_hessian(const Eigen::Vector<T, 3>& a,
                                               const Eigen::Vector<T, 3>& b,
                                               Eigen::Matrix<T, 6, 6>& Hessian);
}  // namespace uipc::backend::cuda::distance

#include "details/point_point.inl"
