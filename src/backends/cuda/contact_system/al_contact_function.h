#pragma once
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

}  // namespace sym::al_simplex_contact
}  // namespace uipc::backend::cuda
