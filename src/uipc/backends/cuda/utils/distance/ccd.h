#pragma once
#include <muda/muda_def.h>
#include <cmath>

//ref: https://github.com/ipc-sim/Codim-IPC/tree/main/Library/Math/Distance
namespace uipc::backend::cuda::distance
{
template <typename T>
MUDA_GENERIC bool point_edge_cd_broadphase(const Eigen::Vector<T, 3>& x0,
                                           const Eigen::Vector<T, 3>& x1,
                                           const Eigen::Vector<T, 3>& x2,
                                           T                          dist);

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Matrix<T, 2, 1>& p,
                                            const Eigen::Matrix<T, 2, 1>& e0,
                                            const Eigen::Matrix<T, 2, 1>& e1,
                                            const Eigen::Matrix<T, 2, 1>& dp,
                                            const Eigen::Matrix<T, 2, 1>& de0,
                                            const Eigen::Matrix<T, 2, 1>& de1,
                                            T                             dist);

template <typename T>
MUDA_GENERIC bool point_triangle_cd_broadphase(const Eigen::Vector<T, 3>& p,
                                               const Eigen::Vector<T, 3>& t0,
                                               const Eigen::Vector<T, 3>& t1,
                                               const Eigen::Vector<T, 3>& t2,
                                               T                          dist);
template <typename T>
MUDA_GENERIC bool edge_edge_cd_broadphase(const Eigen::Vector<T, 3>& ea0,
                                          const Eigen::Vector<T, 3>& ea1,
                                          const Eigen::Vector<T, 3>& eb0,
                                          const Eigen::Vector<T, 3>& eb1,
                                          T                          dist);

template <typename T>
MUDA_GENERIC bool point_triangle_ccd_broadphase(const Eigen::Vector<T, 3>& p,
                                                const Eigen::Vector<T, 3>& t0,
                                                const Eigen::Vector<T, 3>& t1,
                                                const Eigen::Vector<T, 3>& t2,
                                                const Eigen::Vector<T, 3>& dp,
                                                const Eigen::Vector<T, 3>& dt0,
                                                const Eigen::Vector<T, 3>& dt1,
                                                const Eigen::Vector<T, 3>& dt2,
                                                T dist);

template <typename T>
MUDA_GENERIC bool edge_edge_ccd_broadphase(const Eigen::Vector<T, 3>& ea0,
                                           const Eigen::Vector<T, 3>& ea1,
                                           const Eigen::Vector<T, 3>& eb0,
                                           const Eigen::Vector<T, 3>& eb1,
                                           const Eigen::Vector<T, 3>& dea0,
                                           const Eigen::Vector<T, 3>& dea1,
                                           const Eigen::Vector<T, 3>& deb0,
                                           const Eigen::Vector<T, 3>& deb1,
                                           T                          dist);

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Vector<T, 3>& p,
                                            const Eigen::Vector<T, 3>& e0,
                                            const Eigen::Vector<T, 3>& e1,
                                            const Eigen::Vector<T, 3>& dp,
                                            const Eigen::Vector<T, 3>& de0,
                                            const Eigen::Vector<T, 3>& de1,
                                            T                          dist);
template <typename T>
MUDA_GENERIC bool point_point_ccd_broadphase(const Eigen::Vector<T, 3>& p0,
                                             const Eigen::Vector<T, 3>& p1,
                                             const Eigen::Vector<T, 3>& dp0,
                                             const Eigen::Vector<T, 3>& dp1,
                                             T                          dist);

template <typename T>
MUDA_GENERIC bool point_triangle_ccd(Eigen::Vector<T, 3> p,
                                     Eigen::Vector<T, 3> t0,
                                     Eigen::Vector<T, 3> t1,
                                     Eigen::Vector<T, 3> t2,
                                     Eigen::Vector<T, 3> dp,
                                     Eigen::Vector<T, 3> dt0,
                                     Eigen::Vector<T, 3> dt1,
                                     Eigen::Vector<T, 3> dt2,
                                     T                   eta,
                                     T                   thickness,
                                     int                 max_iter,
                                     T&                  toc);

template <typename T>
MUDA_GENERIC bool edge_edge_ccd(Eigen::Vector<T, 3> ea0,
                                Eigen::Vector<T, 3> ea1,
                                Eigen::Vector<T, 3> eb0,
                                Eigen::Vector<T, 3> eb1,
                                Eigen::Vector<T, 3> dea0,
                                Eigen::Vector<T, 3> dea1,
                                Eigen::Vector<T, 3> deb0,
                                Eigen::Vector<T, 3> deb1,
                                T                   eta,
                                T                   thickness,
                                int                 max_iter,
                                T&                  toc);

template <typename T>
MUDA_GENERIC bool point_edge_ccd(Eigen::Vector<T, 3> p,
                                 Eigen::Vector<T, 3> e0,
                                 Eigen::Vector<T, 3> e1,
                                 Eigen::Vector<T, 3> dp,
                                 Eigen::Vector<T, 3> de0,
                                 Eigen::Vector<T, 3> de1,
                                 T                   eta,
                                 T                   thickness,
                                 int                 max_iter,
                                 T&                  toc);
template <typename T>
MUDA_GENERIC bool point_point_ccd(Eigen::Vector<T, 3> p0,
                                  Eigen::Vector<T, 3> p1,
                                  Eigen::Vector<T, 3> dp0,
                                  Eigen::Vector<T, 3> dp1,
                                  T                   eta,
                                  T                   thickness,
                                  int                 max_iter,
                                  T&                  toc);
}  // namespace uipc::backend::cuda::distance

#include "details/ccd.inl"
