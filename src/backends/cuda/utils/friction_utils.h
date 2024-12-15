#pragma once
#include <type_define.h>
#include <muda/ext/eigen/inverse.h>

namespace uipc::backend::cuda::friction
{
template <typename T>
inline UIPC_GENERIC void tangent_rel_dx(const Eigen::Vector<T, 3>&    rel_dx,
                                        const Eigen::Matrix<T, 3, 2>& basis,
                                        Eigen::Vector<T, 2>& tan_rel_dx)
{
    tan_rel_dx = basis.transpose() * rel_dx;
}

// Point - Triangle

template <typename T>
inline UIPC_GENERIC void point_triangle_tangent_basis(const Eigen::Vector<T, 3>& P,
                                                      const Eigen::Vector<T, 3>& T0,
                                                      const Eigen::Vector<T, 3>& T1,
                                                      const Eigen::Vector<T, 3>& T2,
                                                      Eigen::Matrix<T, 3, 2>& basis)
{
    Eigen::Vector<T, 3> v12 = T1 - T0;
    basis.col(0)            = v12.normalized();
    basis.col(1)            = v12.cross(T2 - T0).cross(v12).normalized();
}

template <typename T>
inline UIPC_GENERIC void point_triangle_closest_point(const Eigen::Vector<T, 3>& P,
                                                      const Eigen::Vector<T, 3>& T0,
                                                      const Eigen::Vector<T, 3>& T1,
                                                      const Eigen::Vector<T, 3>& T2,
                                                      Eigen::Vector<T, 2>& beta)
{
    Eigen::Matrix<T, 2, 3> basis;
    basis.row(0)               = (T1 - T0).transpose();
    basis.row(1)               = (T2 - T0).transpose();
    Eigen::Matrix<T, 2, 2> BBT = basis * basis.transpose();
    Eigen::Vector<T, 2>    rhs = basis * (P - T0);

    // Note: don't use Eigen solve, it doesn't work for CUDA
    beta = muda::eigen::inverse(BBT) * rhs;
}

template <typename T>
inline UIPC_GENERIC void point_triangle_rel_dx(const Eigen::Vector<T, 3>& dP,
                                               const Eigen::Vector<T, 3>& dT0,
                                               const Eigen::Vector<T, 3>& dT1,
                                               const Eigen::Vector<T, 3>& dT2,
                                               const Eigen::Vector<T, 2>& beta,
                                               Eigen::Vector<T, 3>& rel_dx)
{
    rel_dx = dP - (dT0 + beta[0] * (dT1 - dT0) + beta[1] * (dT2 - dT0));
}

template <typename T>
inline UIPC_GENERIC void point_triangle_tan_rel_dx(const Eigen::Vector<T, 3>& dP,
                                                   const Eigen::Vector<T, 3>& dT0,
                                                   const Eigen::Vector<T, 3>& dT1,
                                                   const Eigen::Vector<T, 3>& dT2,
                                                   const Eigen::Matrix<T, 3, 2>& basis,
                                                   const Eigen::Vector<T, 2>& beta,
                                                   Eigen::Vector<T, 2>& tan_rel_dx)
{
    Eigen::Vector<T, 3> rel_dx;
    point_triangle_rel_dx(dP, dT0, dT1, dT2, beta, rel_dx);
    tangent_rel_dx(rel_dx, basis, tan_rel_dx);
}

template <typename T>
inline UIPC_GENERIC void apply_point_triangle_jacobi(const Eigen::Vector<T, 2>& G2,
                                                     const Eigen::Matrix<T, 3, 2>& basis,
                                                     const Eigen::Vector<T, 2>& beta,
                                                     Eigen::Matrix<T, 12, 1>& G12)
{
    G12.template segment<3>(0) = basis * G2;
    G12.template segment<3>(3) = (-1 + beta[0] + beta[1]) * G12.template segment<3>(0);
    G12.template segment<3>(6) = -beta[0] * G12.template segment<3>(0);
    G12.template segment<3>(9) = -beta[1] * G12.template segment<3>(0);
}

template <typename T>
inline UIPC_GENERIC void point_triangle_jacobi(const Eigen::Matrix<T, 3, 2>& basis,
                                               const Eigen::Vector<T, 2>& beta,
                                               Eigen::Matrix<T, 2, 12>&   J)
{
    J.template block<2, 3>(0, 0) = basis.transpose();
    J.template block<2, 3>(0, 3) = (-1 + beta[0] + beta[1]) * basis.transpose();
    J.template block<2, 3>(0, 6) = -beta[0] * basis.transpose();
    J.template block<2, 3>(0, 9) = -beta[1] * basis.transpose();
}

// Edge - Edge

template <typename T>
inline UIPC_GENERIC void edge_edge_tangent_basis(const Eigen::Vector<T, 3>& Ea0,
                                                 const Eigen::Vector<T, 3>& Ea1,
                                                 const Eigen::Vector<T, 3>& Eb0,
                                                 const Eigen::Vector<T, 3>& Eb1,
                                                 Eigen::Matrix<T, 3, 2>& basis)
{
    Eigen::Vector<T, 3> v01 = Ea1 - Ea0;
    basis.col(0)            = v01.normalized();
    basis.col(1)            = v01.cross(Eb1 - Eb0).cross(v01).normalized();
}

template <typename T>
inline UIPC_GENERIC void edge_edge_closest_point(const Eigen::Vector<T, 3>& Ea0,
                                                 const Eigen::Vector<T, 3>& Ea1,
                                                 const Eigen::Vector<T, 3>& Eb0,
                                                 const Eigen::Vector<T, 3>& Eb1,
                                                 Eigen::Vector<T, 2>& gamma)
{
    Eigen::Matrix<T, 1, 3> e20 = (Ea0 - Eb0).transpose();
    Eigen::Matrix<T, 1, 3> e01 = (Ea1 - Ea0).transpose();
    Eigen::Matrix<T, 1, 3> e23 = (Eb1 - Eb0).transpose();

    Eigen::Matrix<T, 2, 2> coefMtr;
    coefMtr(0, 0) = e01.squaredNorm();
    coefMtr(0, 1) = coefMtr(1, 0) = -e23.dot(e01);
    coefMtr(1, 1)                 = e23.squaredNorm();

    Eigen::Vector<T, 2> rhs;
    rhs[0] = -e20.dot(e01);
    rhs[1] = e20.dot(e23);

    // Note: don't use Eigen solve, it doesn't work for CUDA
    gamma = muda::eigen::inverse(coefMtr) * rhs;
}

template <typename T>
inline UIPC_GENERIC void edge_edge_rel_dx(const Eigen::Vector<T, 3>& dEa0,
                                          const Eigen::Vector<T, 3>& dEa1,
                                          const Eigen::Vector<T, 3>& dEb0,
                                          const Eigen::Vector<T, 3>& dEb1,
                                          const Eigen::Vector<T, 2>& gamma,
                                          Eigen::Vector<T, 3>&       rel_dx)
{
    rel_dx = dEa0 + gamma[0] * (dEa1 - dEa0) - (dEb0 + gamma[1] * (dEb1 - dEb0));
}

template <typename T>
inline UIPC_GENERIC void edge_edge_tan_rel_dx(const Eigen::Vector<T, 3>& dEa0,
                                              const Eigen::Vector<T, 3>& dEa1,
                                              const Eigen::Vector<T, 3>& dEb0,
                                              const Eigen::Vector<T, 3>& dEb1,
                                              const Eigen::Matrix<T, 3, 2>& basis,
                                              const Eigen::Vector<T, 2>& gamma,
                                              Eigen::Vector<T, 2>& tan_rel_dx)
{
    Eigen::Vector<T, 3> rel_dx;
    edge_edge_rel_dx(dEa0, dEa1, dEb0, dEb1, gamma, rel_dx);
    tangent_rel_dx(rel_dx, basis, tan_rel_dx);
}

template <typename T>
inline UIPC_GENERIC void apply_edge_edge_jacobi(const Eigen::Vector<T, 2>& G2,
                                                const Eigen::Matrix<T, 3, 2>& basis,
                                                const Eigen::Vector<T, 2>& gamma,
                                                Eigen::Vector<T, 12>& G12)
{
    Eigen::Vector<T, 3> relDXTan3D = basis * G2;
    G12.template segment<3>(0)     = (1.0 - gamma[0]) * relDXTan3D;
    G12.template segment<3>(3)     = gamma[0] * relDXTan3D;
    G12.template segment<3>(6)     = (gamma[1] - 1.0) * relDXTan3D;
    G12.template segment<3>(9)     = -gamma[1] * relDXTan3D;
}

template <typename T>
inline UIPC_GENERIC void edge_edge_jacobi(const Eigen::Matrix<T, 3, 2>& basis,
                                          const Eigen::Vector<T, 2>&    gamma,
                                          Eigen::Matrix<T, 2, 12>&      J)
{
    J.template block<2, 3>(0, 0) = (1.0 - gamma[0]) * basis.transpose();
    J.template block<2, 3>(0, 3) = gamma[0] * basis.transpose();
    J.template block<2, 3>(0, 6) = (gamma[1] - 1.0) * basis.transpose();
    J.template block<2, 3>(0, 9) = -gamma[1] * basis.transpose();
}

// Point - Edge

template <typename T>
inline UIPC_GENERIC void point_edge_tangent_basis(const Eigen::Vector<T, 3>& P,
                                                  const Eigen::Vector<T, 3>& E0,
                                                  const Eigen::Vector<T, 3>& E1,
                                                  Eigen::Matrix<T, 3, 2>& basis)
{
    Eigen::Vector<T, 3> v12 = E1 - E0;
    basis.col(0)            = v12.normalized();
    basis.col(1)            = v12.cross(P - E0).normalized();
}

template <typename T>
inline UIPC_GENERIC void point_edge_closest_point(const Eigen::Vector<T, 3>& P,
                                                  const Eigen::Vector<T, 3>& E0,
                                                  const Eigen::Vector<T, 3>& E1,
                                                  T& eta)
{
    Eigen::Vector<T, 3> e12 = E1 - E0;
    eta                     = (P - E0).dot(e12) / e12.squaredNorm();
}

template <typename T>
inline UIPC_GENERIC void point_edge_rel_dx(const Eigen::Vector<T, 3>& dP,
                                           const Eigen::Vector<T, 3>& dE0,
                                           const Eigen::Vector<T, 3>& dE1,
                                           T                          eta,
                                           Eigen::Vector<T, 3>&       rel_dx)
{
    rel_dx = dP - (dE0 + eta * (dE1 - dE0));
}

template <typename T>
inline UIPC_GENERIC void point_edge_tan_rel_dx(const Eigen::Vector<T, 3>& dP,
                                               const Eigen::Vector<T, 3>& dE0,
                                               const Eigen::Vector<T, 3>& dE1,
                                               const Eigen::Matrix<T, 3, 2>& basis,
                                               T                    eta,
                                               Eigen::Vector<T, 2>& tan_rel_dx)
{
    Eigen::Vector<T, 3> rel_dx;
    point_edge_rel_dx(dP, dE0, dE1, eta, rel_dx);
    tangent_rel_dx(rel_dx, basis, tan_rel_dx);
}

template <typename T>
inline UIPC_GENERIC void apply_point_edge_jacobi(const Eigen::Vector<T, 2>& G2,
                                                 const Eigen::Matrix<T, 3, 2>& basis,
                                                 T                    eta,
                                                 Eigen::Vector<T, 9>& G9)
{
    G9.template segment<3>(0) = basis * G2;
    G9.template segment<3>(3) = (eta - 1.0) * G9.template segment<3>(0);
    G9.template segment<3>(6) = -eta * G9.template segment<3>(0);
}

template <typename T>
inline UIPC_GENERIC void point_edge_jacobi(const Eigen::Matrix<T, 3, 2>& basis,
                                           T                             eta,
                                           Eigen::Matrix<T, 2, 9>&       J)
{
    J.template block<2, 3>(0, 0) = basis.transpose();
    J.template block<2, 3>(0, 3) = (eta - 1.0) * basis.transpose();
    J.template block<2, 3>(0, 6) = -eta * basis.transpose();
}

// Point - Point

template <typename T>
inline UIPC_GENERIC void point_point_tangent_basis(const Eigen::Vector<T, 3>& P0,
                                                   const Eigen::Vector<T, 3>& P1,
                                                   Eigen::Matrix<T, 3, 2>& basis)
{
    Eigen::Matrix<T, 1, 3> v01    = (P1 - P0).transpose();
    Eigen::Matrix<T, 1, 3> xCross = Eigen::Matrix<T, 1, 3>::UnitX().cross(v01);
    Eigen::Matrix<T, 1, 3> yCross = Eigen::Matrix<T, 1, 3>::UnitY().cross(v01);
    if(xCross.squaredNorm() > yCross.squaredNorm())
    {
        basis.col(0) = xCross.normalized().transpose();
        basis.col(1) = v01.cross(xCross).normalized().transpose();
    }
    else
    {
        basis.col(0) = yCross.normalized().transpose();
        basis.col(1) = v01.cross(yCross).normalized().transpose();
    }
}

template <typename T>
inline UIPC_GENERIC void point_point_rel_dx(const Eigen::Vector<T, 3>& dP0,
                                            const Eigen::Vector<T, 3>& dP1,
                                            Eigen::Vector<T, 3>&       rel_dx)
{
    rel_dx = dP0 - dP1;
}

template <typename T>
inline UIPC_GENERIC void point_point_tan_rel_dx(const Eigen::Vector<T, 3>& dP0,
                                                const Eigen::Vector<T, 3>& dP1,
                                                const Eigen::Matrix<T, 3, 2>& basis,
                                                Eigen::Vector<T, 2>& tan_rel_dx)
{
    Eigen::Vector<T, 3> rel_dx;
    point_point_rel_dx(dP0, dP1, rel_dx);
    tangent_rel_dx(rel_dx, basis, tan_rel_dx);
}

template <typename T>
inline UIPC_GENERIC void apply_point_point_jacobi(const Eigen::Vector<T, 2>& G2,
                                                  const Eigen::Matrix<T, 3, 2>& basis,
                                                  Eigen::Vector<T, 6>& G6)
{
    G6.template segment<3>(0) = basis * G2;
    G6.template segment<3>(3) = -G6.template segment<3>(0);
}

template <typename T>
inline UIPC_GENERIC void point_point_jacobi(const Eigen::Matrix<T, 3, 2>& basis,
                                            Eigen::Matrix<T, 2, 6>&       J)
{
    J.template block<2, 3>(0, 0) = basis.transpose();
    J.template block<2, 3>(0, 3) = -basis.transpose();
}

}  // namespace uipc::backend::cuda::friction