#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::distance
{
template <typename T>
MUDA_GENERIC void point_edge_distance2(const Eigen::Vector<T, 3>& p,
                                       const Eigen::Vector<T, 3>& e0,
                                       const Eigen::Vector<T, 3>& e1,
                                       T&                         dist2);

template <typename T>
MUDA_GENERIC void point_edge_distance2_gradient(const Eigen::Vector<T, 3>& p,
                                                const Eigen::Vector<T, 3>& e0,
                                                const Eigen::Vector<T, 3>& e1,
                                                Eigen::Vector<T, 9>& grad);

template <typename T>
MUDA_GENERIC void point_edge_distance2_hessian(const Eigen::Vector<T, 3>& p,
                                               const Eigen::Vector<T, 3>& e0,
                                               const Eigen::Vector<T, 3>& e1,
                                               Eigen::Matrix<T, 9, 9>& Hessian);

}  // namespace uipc::backend::cuda::distance

#include "details/point_edge.inl"
