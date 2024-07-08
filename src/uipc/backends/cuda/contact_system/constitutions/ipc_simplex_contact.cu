#include <contact_system/constitutions/ipc_simplex_contact.h>
#include <muda/ext/geo/distance.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexContact);

void IPCSimplexContact::do_build(BuildInfo& info) {}


namespace ipc_contact
{
    namespace sym
    {
#include <contact_system/constitutions/sym/ipc_contact.inl>
    }  // namespace sym


    __device__ Float PT_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P,
                                       const Vector3& T0,
                                       const Vector3& T1,
                                       const Vector3& T2)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);
        Float B;
        sym::KappaBarrier(B, kappa, D, D_hat);
        return B;
    }

    __device__ void PT_barrier_gradient_hessian(Vector12&      G,
                                                Matrix12x12&   H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P,
                                                const Vector3& T0,
                                                const Vector3& T1,
                                                const Vector3& T2)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D;
        point_triangle_distance(P, T0, T1, T2, D);

        Vector12 GradD;
        point_triangle_distance_gradient(P, T0, T1, T2, GradD);

        Float dBdD;
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix12x12 HessD;
        point_triangle_distance_hessian(P, T0, T1, T2, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    __device__ Float EE_barrier_energy(Float          kappa,
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

        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);
        Float B;
        sym::KappaBarrier(B, kappa, D, D_hat);

        Float eps_x;
        edge_edge_mollifier_threshold(t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, eps_x);

        Float ek;
        edge_edge_mollifier(Ea0, Ea1, Eb0, Eb1, eps_x, ek);

        return ek * B;
    }


    __device__ void EE_barrier_gradient_hessian(Vector12&      G,
                                                Matrix12x12&   H,
                                                Float          kappa,
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
        using namespace muda::distance;

        Float D_hat = squared_d_hat;


        Float D;
        edge_edge_distance(Ea0, Ea1, Eb0, Eb1, D);

        Float B;
        sym::KappaBarrier(B, kappa, D, D_hat);

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
        sym::dKappaBarrierdD(dBdD, kappa, D, D_hat);

        //tex: $$ \nabla B = \frac{\partial B}{\partial D} \nabla D$$
        Vector12 GradB = dBdD * GradD;

        //tex:
        //$$
        // G = \nabla e_k B + e_k \nabla B
        //$$
        G = Gradek * B + ek * GradB;

        //tex: $$ \frac{\partial^2 B}{\partial D^2} $$
        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

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

    __device__ Float PE_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P,
                                       const Vector3& E0,
                                       const Vector3& E1)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);
        Float E = 0.0;
        sym::KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    __device__ void PE_barrier_gradient_hessian(Vector9&       G,
                                                Matrix9x9&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P,
                                                const Vector3& E0,
                                                const Vector3& E1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_edge_distance(P, E0, E1, D);

        Vector9 GradD;
        point_edge_distance_gradient(P, E0, E1, GradD);

        Float dBdD;
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix9x9 HessD;
        point_edge_distance_hessian(P, E0, E1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }

    __device__ Float PP_barrier_energy(Float          kappa,
                                       Float          squared_d_hat,
                                       const Vector3& P0,
                                       const Vector3& P1)
    {
        using namespace muda::distance;
        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);
        Float E = 0.0;
        sym::KappaBarrier(E, kappa, D, D_hat);
        return E;
    }

    __device__ void PP_barrier_gradient_hessian(Vector6&       G,
                                                Matrix6x6&     H,
                                                Float          kappa,
                                                Float          squared_d_hat,
                                                const Vector3& P0,
                                                const Vector3& P1)
    {
        using namespace muda::distance;

        Float D_hat = squared_d_hat;
        Float D     = 0.0;
        point_point_distance(P0, P1, D);

        Vector6 GradD;
        point_point_distance_gradient(P0, P1, GradD);

        Float dBdD;
        sym::dKappaBarrierdD(dBdD, kappa, D, squared_d_hat);

        //tex:
        //$$
        // G = \frac{\partial B}{\partial D} \frac{\partial D}{\partial x}
        //$$
        G = dBdD * GradD;

        Float ddBddD;
        sym::ddKappaBarrierddD(ddBddD, kappa, D, squared_d_hat);

        Matrix6x6 HessD;
        point_point_distance_hessian(P0, P1, HessD);

        //tex:
        //$$
        // H = \frac{\partial^2 B}{\partial D^2} \frac{\partial D}{\partial x} \frac{\partial D}{\partial x}^T + \frac{\partial B}{\partial D} \frac{\partial^2 D}{\partial x^2}
        //$$
        H = ddBddD * GradD * GradD.transpose() + dBdD * HessD;
    }
}  // namespace ipc_contact


void IPCSimplexContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    auto PT_count = info.PTs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PT")
        .apply(PT_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PTs   = info.PTs().viewer().name("PTs"),
                Es    = info.PT_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);
                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   // jsut hard coding now
                   auto  kappa = table(0, 0).kappa;
                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_triangle_distance(P, T0, T1, T2, D);

                   MUDA_ASSERT(D < D_hat,
                               "PT[%d,%d,%d,%d] distance is larger than d_hat(%f), %f",
                               PT(0),
                               PT(1),
                               PT(2),
                               PT(3),
                               D_hat,
                               D);

                   Es(i) = ipc_contact::PT_barrier_energy(kappa, d_hat * d_hat, P, T0, T1, T2);

                   MUDA_ASSERT(Es(i) >= 0.0, "PT energy is negative, %f", Es(i));

                   // cout << "PT energy: " << Es(i) << "\n";
               });

    // Compute Edge-Edge energy
    auto EE_count = info.EEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__ "-EE")
        .apply(EE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                EEs   = info.EEs().viewer().name("EEs"),
                Es    = info.EE_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);


                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);

                   MUDA_ASSERT(D < D_hat,
                               "EE[%d,%d,%d,%d] distance is larger than d_hat(%f), %f",
                               EE(0),
                               EE(1),
                               EE(2),
                               EE(3),
                               D_hat,
                               D);

                   Es(i) = ipc_contact::EE_barrier_energy(
                       kappa, d_hat * d_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);

                   MUDA_ASSERT(Es(i) >= 0.0, "EE energy is negative, %f", Es(i));
               });

    // Compute Point-Edge energy
    auto PE_count = info.PEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PE")
        .apply(PE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PEs   = info.PEs().viewer().name("PEs"),
                Es    = info.PE_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);
                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_edge_distance(P, E0, E1, D);

                   MUDA_ASSERT(D < D_hat,
                               "PE[%d,%d,%d] distance is larger than d_hat(%f), %f",
                               PE(0),
                               PE(1),
                               PE(2),
                               D_hat,
                               D);

                   Es(i) = ipc_contact::PE_barrier_energy(kappa, d_hat * d_hat, P, E0, E1);

                   MUDA_ASSERT(Es(i) >= 0.0, "PE energy is negative, %f", Es(i));
               });

    // Compute Point-Point energy
    auto PP_count = info.PPs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PP")
        .apply(PP_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PPs   = info.PPs().viewer().name("PPs"),
                Es    = info.PP_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);
                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_point_distance(P0, P1, D);

                   MUDA_ASSERT(D < D_hat,
                               "PP[%d,%d] distance is larger than d_hat(%f), %f",
                               PP(0),
                               PP(1),
                               D_hat,
                               D);

                   Es(i) = ipc_contact::PP_barrier_energy(kappa, d_hat * d_hat, P0, P1);

                   MUDA_ASSERT(Es(i) >= 0.0, "PP energy is negative, %f", Es(i));
               });
}

void IPCSimplexContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PT")
        .apply(info.PTs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PTs   = info.PTs().viewer().name("PTs"),
                Gs    = info.PT_gradients().viewer().name("Gs"),
                Hs    = info.PT_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);
                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   ipc_contact::PT_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P, T0, T1, T2);
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__ "-EE")
        .apply(info.EEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                EEs   = info.EEs().viewer().name("EEs"),
                Gs    = info.EE_gradients().viewer().name("Gs"),
                Hs    = info.EE_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);
                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   ipc_contact::EE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
               });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PE")
        .apply(info.PEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PEs   = info.PEs().viewer().name("PEs"),
                Gs    = info.PE_gradients().viewer().name("Gs"),
                Hs    = info.PE_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);
                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   ipc_contact::PE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P, E0, E1);
               });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PP")
        .apply(info.PPs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                PPs   = info.PPs().viewer().name("PPs"),
                Gs    = info.PP_gradients().viewer().name("Gs"),
                Hs    = info.PP_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                d_hat = info.d_hat()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);
                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   // jsut hard coding now
                   auto kappa = table(0, 0).kappa;

                   ipc_contact::PP_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P0, P1);
               });
}
}  // namespace uipc::backend::cuda