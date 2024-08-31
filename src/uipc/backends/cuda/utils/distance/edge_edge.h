#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void edge_edge_distance2(const Eigen::Vector<T, 3>& ea0,
                                      const Eigen::Vector<T, 3>& ea1,
                                      const Eigen::Vector<T, 3>& eb0,
                                      const Eigen::Vector<T, 3>& eb1,
                                      T&                         dist2);

template <class T>
MUDA_GENERIC void edge_edge_distance2_gradient(const Eigen::Vector<T, 3>& ea0,
                                               const Eigen::Vector<T, 3>& ea1,
                                               const Eigen::Vector<T, 3>& eb0,
                                               const Eigen::Vector<T, 3>& eb1,
                                               Eigen::Vector<T, 12>&      grad);

template <class T>
MUDA_GENERIC void edge_edge_distance2_hessian(const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1,
                                              Eigen::Matrix<T, 12, 12>& Hessian);

}  // namespace uipc::backend::cuda::distance

#include "details/edge_edge.inl"
