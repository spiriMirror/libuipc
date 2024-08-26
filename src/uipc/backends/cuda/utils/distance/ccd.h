#pragma once
#include <muda/muda_def.h>
#include "distance_type.h"
#include "distance_unclassified.h"
#include <cmath>

//ref: https://github.com/ipc-sim/Codim-IPC/tree/main/Library/Math/Distance
namespace uipc::backend::cuda::distance
{
template <typename T, int dim>
MUDA_GENERIC bool point_edge_cd_broadphase(const Eigen::Matrix<T, dim, 1>& x0,
                                           const Eigen::Matrix<T, dim, 1>& x1,
                                           const Eigen::Matrix<T, dim, 1>& x2,
                                           T dist);

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Matrix<T, 2, 1>& p,
                                            const Eigen::Matrix<T, 2, 1>& e0,
                                            const Eigen::Matrix<T, 2, 1>& e1,
                                            const Eigen::Matrix<T, 2, 1>& dp,
                                            const Eigen::Matrix<T, 2, 1>& de0,
                                            const Eigen::Matrix<T, 2, 1>& de1,
                                            T                             dist);

template <typename T>
MUDA_GENERIC bool point_triangle_cd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                               const Eigen::Matrix<T, 3, 1>& t0,
                                               const Eigen::Matrix<T, 3, 1>& t1,
                                               const Eigen::Matrix<T, 3, 1>& t2,
                                               T dist);
template <typename T>
MUDA_GENERIC bool edge_edge_cd_broadphase(const Eigen::Matrix<T, 3, 1>& ea0,
                                          const Eigen::Matrix<T, 3, 1>& ea1,
                                          const Eigen::Matrix<T, 3, 1>& eb0,
                                          const Eigen::Matrix<T, 3, 1>& eb1,
                                          T                             dist);

template <typename T>
MUDA_GENERIC bool point_triangle_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                                const Eigen::Matrix<T, 3, 1>& t0,
                                                const Eigen::Matrix<T, 3, 1>& t1,
                                                const Eigen::Matrix<T, 3, 1>& t2,
                                                const Eigen::Matrix<T, 3, 1>& dp,
                                                const Eigen::Matrix<T, 3, 1>& dt0,
                                                const Eigen::Matrix<T, 3, 1>& dt1,
                                                const Eigen::Matrix<T, 3, 1>& dt2,
                                                T dist);

template <typename T>
MUDA_GENERIC bool edge_edge_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& ea0,
                                           const Eigen::Matrix<T, 3, 1>& ea1,
                                           const Eigen::Matrix<T, 3, 1>& eb0,
                                           const Eigen::Matrix<T, 3, 1>& eb1,
                                           const Eigen::Matrix<T, 3, 1>& dea0,
                                           const Eigen::Matrix<T, 3, 1>& dea1,
                                           const Eigen::Matrix<T, 3, 1>& deb0,
                                           const Eigen::Matrix<T, 3, 1>& deb1,
                                           T                             dist);

template <typename T>
MUDA_GENERIC bool point_edge_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p,
                                            const Eigen::Matrix<T, 3, 1>& e0,
                                            const Eigen::Matrix<T, 3, 1>& e1,
                                            const Eigen::Matrix<T, 3, 1>& dp,
                                            const Eigen::Matrix<T, 3, 1>& de0,
                                            const Eigen::Matrix<T, 3, 1>& de1,
                                            T                             dist);
template <typename T>
MUDA_GENERIC bool point_point_ccd_broadphase(const Eigen::Matrix<T, 3, 1>& p0,
                                             const Eigen::Matrix<T, 3, 1>& p1,
                                             const Eigen::Matrix<T, 3, 1>& dp0,
                                             const Eigen::Matrix<T, 3, 1>& dp1,
                                             T dist);

template <typename T>
MUDA_GENERIC bool point_triangle_ccd(Eigen::Matrix<T, 3, 1> p,
                                     Eigen::Matrix<T, 3, 1> t0,
                                     Eigen::Matrix<T, 3, 1> t1,
                                     Eigen::Matrix<T, 3, 1> t2,
                                     Eigen::Matrix<T, 3, 1> dp,
                                     Eigen::Matrix<T, 3, 1> dt0,
                                     Eigen::Matrix<T, 3, 1> dt1,
                                     Eigen::Matrix<T, 3, 1> dt2,
                                     T                      eta,
                                     T                      thickness,
                                     int                    max_iter,
                                     T&                     toc);

template <typename T>
MUDA_GENERIC bool edge_edge_ccd(Eigen::Matrix<T, 3, 1> ea0,
                                Eigen::Matrix<T, 3, 1> ea1,
                                Eigen::Matrix<T, 3, 1> eb0,
                                Eigen::Matrix<T, 3, 1> eb1,
                                Eigen::Matrix<T, 3, 1> dea0,
                                Eigen::Matrix<T, 3, 1> dea1,
                                Eigen::Matrix<T, 3, 1> deb0,
                                Eigen::Matrix<T, 3, 1> deb1,
                                T                      eta,
                                T                      thickness,
                                int                    max_iter,
                                T&                     toc);

template <typename T>
MUDA_GENERIC bool point_edge_ccd(const Eigen::Matrix<T, 2, 1>& x0,
                                 const Eigen::Matrix<T, 2, 1>& x1,
                                 const Eigen::Matrix<T, 2, 1>& x2,
                                 const Eigen::Matrix<T, 2, 1>& d0,
                                 const Eigen::Matrix<T, 2, 1>& d1,
                                 const Eigen::Matrix<T, 2, 1>& d2,
                                 T                             eta,
                                 T&                            toc);

template <typename T>
MUDA_GENERIC bool point_edge_ccd(Eigen::Matrix<T, 3, 1> p,
                                 Eigen::Matrix<T, 3, 1> e0,
                                 Eigen::Matrix<T, 3, 1> e1,
                                 Eigen::Matrix<T, 3, 1> dp,
                                 Eigen::Matrix<T, 3, 1> de0,
                                 Eigen::Matrix<T, 3, 1> de1,
                                 T                      eta,
                                 T                      thickness,
                                 int                    max_iter,
                                 T&                     toc);
template <typename T>
MUDA_GENERIC bool point_point_ccd(Eigen::Matrix<T, 3, 1> p0,
                                  Eigen::Matrix<T, 3, 1> p1,
                                  Eigen::Matrix<T, 3, 1> dp0,
                                  Eigen::Matrix<T, 3, 1> dp1,
                                  T                      eta,
                                  T                      thickness,
                                  int                    max_iter,
                                  T&                     toc);
}  // namespace uipc::backend::cuda::distance

#include "details/ccd.inl"
