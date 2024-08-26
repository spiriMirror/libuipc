#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace uipc::backend::cuda::distance
{
template <class T, int dim>
MUDA_GENERIC void point_edge_distance(const Eigen::Vector<T, dim>& p,
                                      const Eigen::Vector<T, dim>& e0,
                                      const Eigen::Vector<T, dim>& e1,
                                      T&                           dist2);

template <class T, int dim>
MUDA_GENERIC void point_edge_distance_gradient(const Eigen::Vector<T, dim>& p,
                                               const Eigen::Vector<T, dim>& e0,
                                               const Eigen::Vector<T, dim>& e1,
                                               Eigen::Vector<T, dim * 3>& grad);

template <class T, int dim>
MUDA_GENERIC void point_edge_distance_hessian(const Eigen::Vector<T, dim>& p,
                                              const Eigen::Vector<T, dim>& e0,
                                              const Eigen::Vector<T, dim>& e1,
                                              Eigen::Matrix<T, dim * 3, dim * 3>& Hessian);

}  // namespace uipc::backend::cuda::distance

#include "details/point_edge.inl"
