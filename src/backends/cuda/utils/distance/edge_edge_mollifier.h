#pragma once
#include <type_define.h>
namespace uipc::backend::cuda::distance
{
/**
 * @brief Compute the mollifier threshold for edge-edge
 * 
 * Default coeff = 1.0e-3.
 */
template <typename T>
MUDA_GENERIC void edge_edge_mollifier_threshold(const Eigen::Vector<T, 3>& ea0_rest,
                                                const Eigen::Vector<T, 3>& ea1_rest,
                                                const Eigen::Vector<T, 3>& eb0_rest,
                                                const Eigen::Vector<T, 3>& eb1_rest,
                                                T& eps_x);

template <typename T>
MUDA_GENERIC void edge_edge_mollifier_threshold(const Eigen::Vector<T, 3>& ea0_rest,
                                                const Eigen::Vector<T, 3>& ea1_rest,
                                                const Eigen::Vector<T, 3>& eb0_rest,
                                                const Eigen::Vector<T, 3>& eb1_rest,
                                                Float coeff,
                                                T&    eps_x);

template <typename T>
MUDA_GENERIC bool need_mollify(const Eigen::Vector<T, 3>& ea0,
                               const Eigen::Vector<T, 3>& ea1,
                               const Eigen::Vector<T, 3>& eb0,
                               const Eigen::Vector<T, 3>& eb1,
                               T                          eps_x);

template <typename T>
MUDA_GENERIC void edge_edge_cross_norm2(const Eigen::Vector<T, 3>& ea0,
                                        const Eigen::Vector<T, 3>& ea1,
                                        const Eigen::Vector<T, 3>& eb0,
                                        const Eigen::Vector<T, 3>& eb1,
                                        T&                         result);

template <typename T>
MUDA_GENERIC void edge_edge_cross_norm2_gradient(const Eigen::Vector<T, 3>& ea0,
                                                 const Eigen::Vector<T, 3>& ea1,
                                                 const Eigen::Vector<T, 3>& eb0,
                                                 const Eigen::Vector<T, 3>& eb1,
                                                 Eigen::Vector<T, 12>& grad);

template <typename T>
MUDA_GENERIC void edge_edge_cross_norm2_hessian(const Eigen::Vector<T, 3>& ea0,
                                                const Eigen::Vector<T, 3>& ea1,
                                                const Eigen::Vector<T, 3>& eb0,
                                                const Eigen::Vector<T, 3>& eb1,
                                                Eigen::Matrix<T, 12, 12>& Hessian);

template <typename T>
MUDA_GENERIC void edge_edge_mollifier(const Eigen::Vector<T, 3>& ea0,
                                      const Eigen::Vector<T, 3>& ea1,
                                      const Eigen::Vector<T, 3>& eb0,
                                      const Eigen::Vector<T, 3>& eb1,
                                      T                          eps_x,
                                      T&                         e);

template <typename T>
MUDA_GENERIC void edge_edge_mollifier_gradient(const Eigen::Vector<T, 3>& ea0,
                                               const Eigen::Vector<T, 3>& ea1,
                                               const Eigen::Vector<T, 3>& eb0,
                                               const Eigen::Vector<T, 3>& eb1,
                                               T                          eps_x,
                                               Eigen::Vector<T, 12>&      g);

template <typename T>
MUDA_GENERIC void edge_edge_mollifier_hessian(const Eigen::Vector<T, 3>& ea0,
                                              const Eigen::Vector<T, 3>& ea1,
                                              const Eigen::Vector<T, 3>& eb0,
                                              const Eigen::Vector<T, 3>& eb1,
                                              T                          eps_x,
                                              Eigen::Matrix<T, 12, 12>&  H);
}  // namespace uipc::backend::cuda::distance

#include "details/edge_edge_mollifier.inl"
