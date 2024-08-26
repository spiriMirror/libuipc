#pragma once
#include <muda/muda_def.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>

namespace uipc::backend::cuda::distance
{
template <class T>
MUDA_GENERIC void edge_edge_cross_norm2(const Eigen::Vector<T, 3>& ea0,
                                        const Eigen::Vector<T, 3>& ea1,
                                        const Eigen::Vector<T, 3>& eb0,
                                        const Eigen::Vector<T, 3>& eb1,
                                        T&                         result);

template <class T>
MUDA_GENERIC void edge_edge_cross_norm2_gradient(const Eigen::Vector<T, 3>& ea0,
                                                 const Eigen::Vector<T, 3>& ea1,
                                                 const Eigen::Vector<T, 3>& eb0,
                                                 const Eigen::Vector<T, 3>& eb1,
                                                 Eigen::Vector<T, 12>& grad);

template <class T>
MUDA_GENERIC void edge_edge_cross_norm2_hessian(const Eigen::Vector<T, 3>& ea0,
                                                const Eigen::Vector<T, 3>& ea1,
                                                const Eigen::Vector<T, 3>& eb0,
                                                const Eigen::Vector<T, 3>& eb1,
                                                Eigen::Matrix<T, 12, 12>& Hessian);

template <class T>
MUDA_GENERIC void edge_edge_mollifier(const Eigen::Vector<T, 3>& ea0,
                                      const Eigen::Vector<T, 3>& ea1,
                                      const Eigen::Vector<T, 3>& eb0,
                                      const Eigen::Vector<T, 3>& eb1,
                                      T                          eps_x,
                                      T&                         e);

template <class T>
MUDA_GENERIC void edge_edge_mollifier_gradient(const Eigen::Vector<T, 3>& ea0,
                                               const Eigen::Vector<T, 3>& ea1,
                                               const Eigen::Vector<T, 3>& eb0,
                                               const Eigen::Vector<T, 3>& eb1,
                                               T                          eps_x,
                                               Eigen::Vector<T, 12>&      g);

template <class T>
MUDA_GENERIC void edge_edge_mollifier_hessian(const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1,
                                              T                          eps_x,
                                              Eigen::Matrix<T, 12, 12>&  H);

template <class T>
MUDA_GENERIC void edge_edge_mollifier_threshold(const Eigen::Vector<T, 3>& ea0_rest,
                                                const Eigen::Vector<T, 3>& ea1_rest,
                                                const Eigen::Vector<T, 3>& eb0_rest,
                                                const Eigen::Vector<T, 3>& eb1_rest,
                                                T& eps_x);
}  // namespace uipc::backend::cuda::distance

#include "details/edge_edge_mollified.inl"
