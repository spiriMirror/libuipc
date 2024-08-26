#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void point_triangle_distance(const Eigen::Vector<T, 3>& p,
                                          const Eigen::Vector<T, 3>& t0,
                                          const Eigen::Vector<T, 3>& t1,
                                          const Eigen::Vector<T, 3>& t2,
                                          T&                         dist2);

template <class T>
MUDA_GENERIC void point_triangle_distance_gradient(const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& t0,
                                                   const Eigen::Vector<T, 3>& t1,
                                                   const Eigen::Vector<T, 3>& t2,
                                                   Eigen::Vector<T, 12>& grad);

template <class T>
MUDA_GENERIC void point_triangle_distance_hessian(const Eigen::Vector<T, 3>& p,
                                                  const Eigen::Vector<T, 3>& t0,
                                                  const Eigen::Vector<T, 3>& t1,
                                                  const Eigen::Vector<T, 3>& t2,
                                                  Eigen::Matrix<T, 12, 12>& Hessian);
}  // namespace uipc::backend::cuda::distance

#include "details/point_triangle.inl"
