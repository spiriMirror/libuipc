#pragma once
#include <contact_system/contact_models/ipc_contact_function.h>

namespace uipc::backend::cuda
{
namespace sym::ipc_vertex_half_contact
{
#include "sym/vertex_half_plane_distance.inl"

    __device__ Float PH_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& v,
                                       const Vector3& P,
                                       const Vector3& N)
    {
        using namespace ipc_contact;

        Float D_hat = squared_d_hat;
        Float D;
        HalfPlaneD(D, v, P, N);

        MUDA_ASSERT(D < D_hat, "D=%f,D_hat=%f, why?", D, D_hat);

        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);

        return E;
    }

    __device__ void PH_barrier_gradient_hessian(Vector3&       G,
                                                Matrix3x3&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& v,
                                                const Vector3& P,
                                                const Vector3& N)
    {
        using namespace ipc_contact;

        Float D_hat = squared_d_hat;
        Float D;
        HalfPlaneD(D, v, P, N);

        MUDA_ASSERT(D < D_hat, "D=%f,D_hat=%f, why?", D, D_hat);

        Float dBdD = 0.0;
        dKappaBarrierdD(dBdD, kappa, D, D_hat);

        Vector3 dDdx;
        dHalfPlaneDdx(dDdx, v, P, N);

        G = dBdD * dDdx;

        Float ddBddD = 0.0;
        ddKappaBarrierddD(ddBddD, kappa, D, D_hat);

        Matrix3x3 ddDddx;
        ddHalfPlaneDddx(ddDddx, v, P, N);

        H = ddBddD * dDdx * dDdx.transpose() + dBdD * ddDddx;
    }
}  // namespace sym::ipc_vertex_half_contact
}  // namespace uipc::backend::cuda
