#pragma once
#include <type_define.h>
#include <contact_system/contact_models/codim_ipc_contact_function.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>

namespace uipc::backend::cuda
{
namespace sym::ipc_vertex_half_contact
{
#include "sym/vertex_half_plane_distance.inl"

    inline __device__ Float PH_barrier_energy(Float          kappa,
                                              Float          d_hat,
                                              Float          thickness,
                                              const Vector3& v,
                                              const Vector3& P,
                                              const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float D;
        HalfPlaneD(D, v, P, N);
        Float E = 0.0;
        KappaBarrier(E, kappa, D, d_hat, thickness);

        return E;
    }

    inline __device__ void PH_barrier_gradient_hessian(Vector3&       G,
                                                       Matrix3x3&     H,
                                                       Float          kappa,
                                                       Float          d_hat,
                                                       Float          thickness,
                                                       const Vector3& v,
                                                       const Vector3& P,
                                                       const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float D;
        HalfPlaneD(D, v, P, N);

        Float dBdD = 0.0;
        dKappaBarrierdD(dBdD, kappa, D, d_hat, thickness);

        Vector3 dDdx;
        dHalfPlaneDdx(dDdx, v, P, N);

        G = dBdD * dDdx;

        Float ddBddD = 0.0;
        ddKappaBarrierddD(ddBddD, kappa, D, d_hat, thickness);

        Matrix3x3 ddDddx;
        ddHalfPlaneDddx(ddDddx, v, P, N);

        H = ddBddD * dDdx * dDdx.transpose() + dBdD * ddDddx;
    }

    inline __device__ void PH_barrier_gradient(Vector3&       G,
                                               Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               const Vector3& v,
                                               const Vector3& P,
                                               const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float D;
        HalfPlaneD(D, v, P, N);

        Float dBdD = 0.0;
        dKappaBarrierdD(dBdD, kappa, D, d_hat, thickness);

        Vector3 dDdx;
        dHalfPlaneDdx(dDdx, v, P, N);

        G = dBdD * dDdx;
    }

    inline __device__ void compute_tan_basis(Vector3& e1, Vector3& e2, const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Vector3 trial = Vector3::UnitX();
        if(N.dot(trial) > 0.9)
        {
            trial = Vector3::UnitZ();
            e1    = trial.cross(N).normalized();
        }
        else
        {
            e1 = trial.cross(N).normalized();
        }
        e2 = N.cross(e1);
    }


    inline __device__ Float PH_friction_energy(Float          kappa,
                                               Float          d_hat,
                                               Float          thickness,
                                               Float          mu,
                                               Float          eps_vh,
                                               const Vector3& prev_v,
                                               const Vector3& v,
                                               const Vector3& P,
                                               const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float prev_D;
        HalfPlaneD(prev_D, prev_v, P, N);
        Float f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dV = v - prev_v;


        Vector3 e1, e2;
        compute_tan_basis(e1, e2, N);

        Vector2 tan_dV;

        TR(tan_dV, v, prev_v, e1, e2);

        return friction_energy(mu, f, eps_vh, tan_dV);
    }

    inline __device__ void PH_friction_gradient_hessian(Vector3&   G,
                                                        Matrix3x3& H,
                                                        Float      kappa,
                                                        Float      d_hat,
                                                        Float      thickness,
                                                        Float      mu,
                                                        Float      eps_vh,
                                                        const Vector3& prev_v,
                                                        const Vector3& v,
                                                        const Vector3& P,
                                                        const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float prev_D;
        HalfPlaneD(prev_D, prev_v, P, N);
        Float f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dV = v - prev_v;

        Vector3 e1, e2;
        compute_tan_basis(e1, e2, N);

        Vector2 tan_dV;

        TR(tan_dV, v, prev_v, e1, e2);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_dV);

        Matrix<Float, 2, 3> J;
        dTRdx(J, v, prev_v, e1, e2);

        G = J.transpose() * G2;

        Matrix2x2 H2x2;
        friction_hessian(H2x2, mu, f, eps_vh, tan_dV);

        H = J.transpose() * H2x2 * J;
    }

    inline __device__ void PH_friction_gradient(Vector3&   G,
                                                Float      kappa,
                                                Float      d_hat,
                                                Float      thickness,
                                                Float      mu,
                                                Float      eps_vh,
                                                const Vector3& prev_v,
                                                const Vector3& v,
                                                const Vector3& P,
                                                const Vector3& N)
    {
        using namespace codim_ipc_contact;

        Float prev_D;
        HalfPlaneD(prev_D, prev_v, P, N);
        Float f = normal_force(kappa, d_hat, thickness, prev_D);

        Vector3 dV = v - prev_v;

        Vector3 e1, e2;
        compute_tan_basis(e1, e2, N);

        Vector2 tan_dV;
        TR(tan_dV, v, prev_v, e1, e2);

        Vector2 G2;
        friction_gradient(G2, mu, f, eps_vh, tan_dV);

        Matrix<Float, 2, 3> J;
        dTRdx(J, v, prev_v, e1, e2);

        G = J.transpose() * G2;
    }
}  // namespace sym::ipc_vertex_half_contact
}  // namespace uipc::backend::cuda
