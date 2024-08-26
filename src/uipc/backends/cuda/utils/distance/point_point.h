#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace uipc::backend::cuda::distance
{
template <class T, int dim>
MUDA_GENERIC void point_point_distance(const Eigen::Vector<T, dim>& a,
                                       const Eigen::Vector<T, dim>& b,
                                       T&                           dist2);

template <class T, int dim>
MUDA_GENERIC void point_point_distance_gradient(const Eigen::Vector<T, dim>& a,
                                                const Eigen::Vector<T, dim>& b,
                                                Eigen::Vector<T, dim * 2>& grad);

template <class T, int dim>
MUDA_GENERIC void point_point_distance_hessian(const Eigen::Vector<T, dim>& a,
                                               const Eigen::Vector<T, dim>& b,
                                               Eigen::Matrix<T, dim * 2, dim * 2>& Hessian);
}  // namespace uipc::backend::cuda::distance

#include "details/point_point.inl"
