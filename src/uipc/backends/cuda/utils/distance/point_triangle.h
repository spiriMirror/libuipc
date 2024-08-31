#pragma once
#include <type_define.h>

namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void point_triangle_distance2(const Eigen::Vector<T, 3>& p,
                                           const Eigen::Vector<T, 3>& t0,
                                           const Eigen::Vector<T, 3>& t1,
                                           const Eigen::Vector<T, 3>& t2,
                                           T&                         dist2);

template <class T>
MUDA_GENERIC void point_triangle_distance2_gradient(const Eigen::Vector<T, 3>& p,
                                                    const Eigen::Vector<T, 3>& t0,
                                                    const Eigen::Vector<T, 3>& t1,
                                                    const Eigen::Vector<T, 3>& t2,
                                                    Eigen::Vector<T, 12>& grad);

template <class T>
MUDA_GENERIC void point_triangle_distance2_hessian(const Eigen::Vector<T, 3>& p,
                                                   const Eigen::Vector<T, 3>& t0,
                                                   const Eigen::Vector<T, 3>& t1,
                                                   const Eigen::Vector<T, 3>& t2,
                                                   Eigen::Matrix<T, 12, 12>& Hessian);
}  // namespace uipc::backend::cuda::distance

#include "details/point_triangle.inl"
