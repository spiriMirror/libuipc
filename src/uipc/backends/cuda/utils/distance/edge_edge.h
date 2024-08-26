#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void edge_edge_distance(const Eigen::Vector<T, 3>& ea0,
                                     const Eigen::Vector<T, 3>& ea1,
                                     const Eigen::Vector<T, 3>& eb0,
                                     const Eigen::Vector<T, 3>& eb1,
                                     T&                         dist2);

template <class T>
MUDA_GENERIC void edge_edge_distance_gradient(const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1,
                                              Eigen::Vector<T, 12>&      grad);

template <class T>
MUDA_GENERIC void edge_edge_distance_hessian(const Eigen::Vector<T, 3>& ea0,
                                             const Eigen::Vector<T, 3>& ea1,
                                             const Eigen::Vector<T, 3>& eb0,
                                             const Eigen::Vector<T, 3>& eb1,
                                             Eigen::Matrix<T, 12, 12>& Hessian);

}  // namespace uipc::backend::cuda::distance

#include "details/edge_edge.inl"
