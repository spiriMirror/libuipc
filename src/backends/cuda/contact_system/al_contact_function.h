#pragma once
#include <contact_system/contact_models/codim_ipc_contact_function.h>
#include <utils/friction_utils.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::al_simplex_contact
{
    inline UIPC_GENERIC Float penalty_energy(Float           scale,
                                             Float           d0,
                                             const Vector12& d_grad,
                                             const Vector3&  P0,
                                             const Vector3&  P1,
                                             const Vector3&  P2,
                                             const Vector3&  P3)
    {
        Float d = d0;
        d += d_grad.segment<3>(0).dot(P0);
        d += d_grad.segment<3>(3).dot(P1);
        d += d_grad.segment<3>(6).dot(P2);
        d += d_grad.segment<3>(9).dot(P3);
        return 0.5 * scale * d * d;
    }

    inline UIPC_GENERIC void penalty_gradient_hessian(Float           scale,
                                                      Float           d0,
                                                      const Vector12& d_grad,
                                                      const Vector3&  P0,
                                                      const Vector3&  P1,
                                                      const Vector3&  P2,
                                                      const Vector3&  P3,
                                                      Vector12&       G,
                                                      Matrix12x12&    H)
    {
        Float d = d0;
        d += d_grad.segment<3>(0).dot(P0);
        d += d_grad.segment<3>(3).dot(P1);
        d += d_grad.segment<3>(6).dot(P2);
        d += d_grad.segment<3>(9).dot(P3);
        G = scale * d * d_grad;
        H = scale * d_grad * d_grad.transpose();
    }

    inline UIPC_DEVICE Float PT_friction_energy(Float          mu,
                                                Float          eps_vh,
                                                Float          normal_force,
                                                const Vector3& prev_P,
                                                const Vector3& prev_T0,
                                                const Vector3& prev_T1,
                                                const Vector3& prev_T2,
                                                const Vector3& P,
                                                const Vector3& T0,
                                                const Vector3& T1,
                                                const Vector3& T2)
    {
        using namespace friction;
        using namespace codim_ipc_contact;

        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        point_triangle_closest_point(prev_P, prev_T0, prev_T1, prev_T2, beta);
        point_triangle_tangent_basis(prev_P, prev_T0, prev_T1, prev_T2, basis);

        Vector3 dP  = P - prev_P;
        Vector3 dT0 = T0 - prev_T0;
        Vector3 dT1 = T1 - prev_T1;
        Vector3 dT2 = T2 - prev_T2;
        point_triangle_tan_rel_dx(dP, dT0, dT1, dT2, basis, beta, tan_rel_dx);

        Float E = friction_energy(mu, normal_force, eps_vh, tan_rel_dx);
        return E;
    }

    inline UIPC_DEVICE void PT_friction_gradient_hessian(Vector12&    G,
                                                         Matrix12x12& H,
                                                         Float        mu,
                                                         Float        eps_vh,
                                                         Float normal_force,
                                                         const Vector3& prev_P,
                                                         const Vector3& prev_T0,
                                                         const Vector3& prev_T1,
                                                         const Vector3& prev_T2,
                                                         const Vector3& P,
                                                         const Vector3& T0,
                                                         const Vector3& T1,
                                                         const Vector3& T2)
    {
        using namespace friction;
        using namespace codim_ipc_contact;

        Vector2             beta;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        point_triangle_closest_point(prev_P, prev_T0, prev_T1, prev_T2, beta);
        point_triangle_tangent_basis(prev_P, prev_T0, prev_T1, prev_T2, basis);

        Vector3 dP  = P - prev_P;
        Vector3 dT0 = T0 - prev_T0;
        Vector3 dT1 = T1 - prev_T1;
        Vector3 dT2 = T2 - prev_T2;
        point_triangle_tan_rel_dx(dP, dT0, dT1, dT2, basis, beta, tan_rel_dx);

        Matrix<Float, 2, 12> J;
        point_triangle_jacobi(basis, beta, J);

        Vector2 G2;
        friction_gradient(G2, mu, normal_force, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, normal_force, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }

    inline UIPC_DEVICE Float EE_friction_energy(Float          mu,
                                                Float          eps_vh,
                                                Float          normal_force,
                                                const Vector3& prev_Ea0,
                                                const Vector3& prev_Ea1,
                                                const Vector3& prev_Eb0,
                                                const Vector3& prev_Eb1,
                                                const Vector3& Ea0,
                                                const Vector3& Ea1,
                                                const Vector3& Eb0,
                                                const Vector3& Eb1)
    {
        using namespace friction;
        using namespace codim_ipc_contact;

        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        edge_edge_closest_point(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, gamma);
        edge_edge_tangent_basis(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, basis);

        Vector3 dEa0 = Ea0 - prev_Ea0;
        Vector3 dEa1 = Ea1 - prev_Ea1;
        Vector3 dEb0 = Eb0 - prev_Eb0;
        Vector3 dEb1 = Eb1 - prev_Eb1;
        edge_edge_tan_rel_dx(dEa0, dEa1, dEb0, dEb1, basis, gamma, tan_rel_dx);

        Float E = friction_energy(mu, normal_force, eps_vh, tan_rel_dx);
        return E;
    }

    inline UIPC_DEVICE void EE_friction_gradient_hessian(Vector12&    G,
                                                         Matrix12x12& H,
                                                         Float        mu,
                                                         Float        eps_vh,
                                                         Float normal_force,
                                                         const Vector3& prev_Ea0,
                                                         const Vector3& prev_Ea1,
                                                         const Vector3& prev_Eb0,
                                                         const Vector3& prev_Eb1,
                                                         const Vector3& Ea0,
                                                         const Vector3& Ea1,
                                                         const Vector3& Eb0,
                                                         const Vector3& Eb1)
    {
        using namespace friction;
        using namespace codim_ipc_contact;

        Vector2             gamma;
        Matrix<Float, 3, 2> basis;
        Vector2             tan_rel_dx;

        edge_edge_closest_point(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, gamma);
        edge_edge_tangent_basis(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, basis);

        Vector3 dEa0 = Ea0 - prev_Ea0;
        Vector3 dEa1 = Ea1 - prev_Ea1;
        Vector3 dEb0 = Eb0 - prev_Eb0;
        Vector3 dEb1 = Eb1 - prev_Eb1;
        edge_edge_tan_rel_dx(dEa0, dEa1, dEb0, dEb1, basis, gamma, tan_rel_dx);

        Matrix<Float, 2, 12> J;
        point_triangle_jacobi(basis, gamma, J);

        Vector2 G2;
        friction_gradient(G2, mu, normal_force, eps_vh, tan_rel_dx);
        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, normal_force, eps_vh, tan_rel_dx);
        H = J.transpose() * H2x2 * J;
    }
}  // namespace sym::al_simplex_contact
}  // namespace uipc::backend::cuda
