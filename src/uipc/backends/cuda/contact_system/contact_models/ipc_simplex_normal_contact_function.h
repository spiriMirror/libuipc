#pragma once
#include <type_define.h>
#include <utils/distance.h>
#include <contact_system/contact_models/ipc_contact_function.h>

namespace uipc::backend::cuda
{
namespace sym::ipc_simplex_contact
{
    inline __device__ Float PT_barrier_energy(Float          kappa,
                                              Float          squared_d_hat,
                                              const Vector3& P,
                                              const Vector3& T0,
                                              const Vector3& T1,
                                              const Vector3& T2)
    {
        using namespace ipc_contact;
        using namespace distance;
        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);
        Float B;
        KappaBarrier(B, kappa, D, D_hat);
        return B;
    }

    inline __device__ void PT_barrier_gradient_hessian(Vector12&    G,
                                                       Matrix12x12& H,
                                                       Float        kappa,
                                                       Float squared_d_hat,
                                                       const Vector3& P,
                                                       const Vector3& T0,
                                                       const Vector3& T1,
                                                       const Vector3& T2)
    {
        using namespace ipc_contact;
        using namespace distance;

        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);

        Vector12 GradD;
        point_triangle_distance_gradient(P, T0, T1, T2, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix12x12 HessD;
        point_triangle_distance_hessian(P, T0, T1, T2, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    inline __device__ Float EE_barrier_energy(Float          kappa,
                                              Float          squared_d_hat,
                                              const Vector3& t0_Ea0,
                                              const Vector3& t0_Ea1,
                                              const Vector3& t0_Eb0,
                                              const Vector3& t0_Eb1,
                                              const Vector3& Ea0,
                                              const Vector3& Ea1,
                                              const Vector3& Eb0,
                                              const Vector3& Eb1)
    {
        // using mollifier to improve the smoothness of the edge-edge barrier
        using namespace ipc_contact;
        using namespace distance;
        Float D_hat = squared_d_hat;
        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);
        Float B;
        KappaBarrier(B, kappa, D, D_hat);

        Float eps_x;
        edge_edge_mollifier_threshold(t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, eps_x);

        Float ek;
        edge_edge_mollifier(Ea0, Ea1, Eb0, Eb1, eps_x, ek);

        return ek * B;
    }


    inline __device__ void EE_barrier_gradient_hessian(Vector12&    G,
                                                       Matrix12x12& H,
                                                       Float        kappa,
                                                       Float squared_d_hat,
                                                       const Vector3& t0_Ea0,
                                                       const Vector3& t0_Ea1,
                                                       const Vector3& t0_Eb0,
                                                       const Vector3& t0_Eb1,
                                                       const Vector3& Ea0,
                                                       const Vector3& Ea1,
                                                       const Vector3& Eb0,
                                                       const Vector3& Eb1)
    {
        using namespace ipc_contact;
        using namespace distance;

        Float D_hat = squared_d_hat;


        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);

        Float B;
        KappaBarrier(B, kappa, D, D_hat);

        //tex: $$ \epsilon_x $$
        Float eps_x;
        edge_edge_mollifier_threshold(t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, eps_x);

        //tex: $$ e_k $$
        Float ek;
        edge_edge_mollifier(Ea0, Ea1, Eb0, Eb1, eps_x, ek);

        //tex: $$\nabla e_k$$
        Vector12 Gradek;
        edge_edge_mollifier_gradient(Ea0, Ea1, Eb0, Eb1, eps_x, Gradek);

        //tex: $$ \nabla D$$
        Vector12 GradD;
        edge_edge_distance_gradient(Ea0, Ea1, Eb0, Eb1, GradD);

        //tex: $$ \frac{\partial B}{\partial D} $$
        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, D_hat);

        //tex: $$ \nabla B = \frac{\partial B}{\partial D} \nabla D$$
        Vector12 GradB = dBdD * GradD;

        //tex:
        //$$
        // G = \nabla e_k B + e_k \nabla B
        //$$
        G = Gradek * B + ek * GradB;

        //tex: $$ \frac{\partial^2 B}{\partial D^2} $$
        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        //tex: $$ \nabla^2 D$$
        Matrix12x12 HessD;
        edge_edge_distance_hessian(Ea0, Ea1, Eb0, Eb1, HessD);

        //tex:
        //$$
        // \nabla^2 B = \frac{\partial^2 B}{\partial D^2} \nabla D \nabla D^T + \frac{\partial B}{\partial D} \nabla^2 D
        //$$
        Matrix12x12 HessB = ddBddD * GradD * GradD.transpose() + dBdD * HessD;

        //tex: $$ \nabla^2 e_k$$
        Matrix12x12 Hessek;
        edge_edge_mollifier_hessian(Ea0, Ea1, Eb0, Eb1, eps_x, Hessek);

        //tex: $$ \nabla^2 e_k B + \nabla e_k \nabla B^T + \nabla B \nabla e_k^T + e_k \nabla^2 B$$
        H = Hessek * B + Gradek * GradB.transpose() + GradB * Gradek.transpose() + ek * HessB;
    }

    inline __device__ Float PE_barrier_energy(Float          kappa,
                                              Float          squared_d_hat,
                                              const Vector3& P,
                                              const Vector3& E0,
                                              const Vector3& E1)
    {
        using namespace ipc_contact;
        using namespace distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);
        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    inline __device__ void PE_barrier_gradient_hessian(Vector9&   G,
                                                       Matrix9x9& H,
                                                       Float      kappa,
                                                       Float      squared_d_hat,
                                                       const Vector3& P,
                                                       const Vector3& E0,
                                                       const Vector3& E1)
    {
        using namespace ipc_contact;
        using namespace distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);

        Vector9 GradD;
        point_edge_distance_gradient(P, E0, E1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix9x9 HessD;
        point_edge_distance_hessian(P, E0, E1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    inline __device__ Float PP_barrier_energy(Float          kappa,
                                              Float          squared_d_hat,
                                              const Vector3& P0,
                                              const Vector3& P1)
    {
        using namespace ipc_contact;
        using namespace distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);
        Float E = 0.0;
        KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    inline __device__ void PP_barrier_gradient_hessian(Vector6&   G,
                                                       Matrix6x6& H,
                                                       Float      kappa,
                                                       Float      squared_d_hat,
                                                       const Vector3& P0,
                                                       const Vector3& P1)
    {
        using namespace ipc_contact;
        using namespace distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);

        Vector6 GradD;
        point_point_distance_gradient(P0, P1, GradD);

        Float dBdD;
        dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix6x6 HessD;
        point_point_distance_hessian(P0, P1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }
}  // namespace sym::ipc_simplex_contact
}  // namespace uipc::backend::cuda
