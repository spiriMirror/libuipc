#pragma once
#include <contact_system/contact_models/ipc_vertex_half_plane_contact_function.h>
#include <utils/friction_utils.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::al_vertex_half_plane_contact
{
    inline UIPC_GENERIC Float half_plane_penalty_energy(Float          scale,
                                                        Float          d0,
                                                        const Vector3& d_grad,
                                                        const Vector3& P0)
    {
        Float d = d0;
        d += d_grad.dot(P0);
        return 0.5 * scale * d * d;
    }

    inline UIPC_GENERIC void half_plane_penalty_gradient_hessian(
        Float scale, Float d0, const Vector3& d_grad, const Vector3& P, Vector3& G, Matrix3x3& H)
    {
        Float d = d0;
        d += d_grad.dot(P);
        G = scale * d * d_grad;
        H = scale * d_grad * d_grad.transpose();
    }

    inline UIPC_DEVICE Float half_plane_frictional_energy(Float mu,
                                                          Float eps_vh,
                                                          Float normal_force,
                                                          const Vector3& v,
                                                          const Vector3& prev_v,
                                                          const Vector3& N)
    {
        using namespace codim_ipc_contact;
        using namespace ipc_vertex_half_contact;

        Vector3 e1, e2;
        compute_tan_basis(e1, e2, N);

        Vector2 tan_dV;
        TR(tan_dV, v, prev_v, e1, e2);

        return friction_energy(mu, normal_force, eps_vh, tan_dV);
    }

    inline UIPC_DEVICE void half_plane_frictional_gradient_hessian(Vector3&   G,
                                                                   Matrix3x3& H,
                                                                   Float mu,
                                                                   Float eps_vh,
                                                                   Float normal_force,
                                                                   const Vector3& v,
                                                                   const Vector3& prev_v,
                                                                   const Vector3& N)
    {
        using namespace codim_ipc_contact;
        using namespace ipc_vertex_half_contact;

        Vector3 e1, e2;
        compute_tan_basis(e1, e2, N);

        Vector2 tan_dV;
        TR(tan_dV, v, prev_v, e1, e2);

        Vector2 G2;
        friction_gradient(G2, mu, normal_force, eps_vh, tan_dV);

        Matrix<Float, 2, 3> J;
        dTRdx(J, v, prev_v, e1, e2);

        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, normal_force, eps_vh, tan_dV);

        H = J.transpose() * H2x2 * J;
    }

}  // namespace sym::al_vertex_half_plane_contact
}  // namespace uipc::backend::cuda
