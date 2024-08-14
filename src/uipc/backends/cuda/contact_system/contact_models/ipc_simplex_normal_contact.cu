#include <contact_system/contact_models/ipc_simplex_normal_contact.h>
#include <contact_system/contact_models/ipc_simplex_normal_contact_function.h>
#include <muda/ext/geo/distance.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexNormalContact);

void IPCSimplexNormalContact::do_build(BuildInfo& info) {}

void IPCSimplexNormalContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    auto PT_count = info.PTs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PT_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs   = info.PTs().viewer().name("PTs"),
                Es    = info.PT_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;


                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_triangle_distance(P, T0, T1, T2, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PT[%d,%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               PT(0),
                               PT(1),
                               PT(2),
                               PT(3),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PT_barrier_energy(
                       kappa, D_hat, P, T0, T1, T2);
               });

    // Compute Edge-Edge energy
    auto EE_count = info.EEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(EE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs     = info.EEs().viewer().name("EEs"),
                Es      = info.EE_energies().viewer().name("Es"),
                Ps      = info.positions().viewer().name("Ps"),
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "EE[%d,%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               EE(0),
                               EE(1),
                               EE(2),
                               EE(3),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::EE_barrier_energy(
                       kappa, D_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
               });

    // Compute Point-Edge energy
    auto PE_count = info.PEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs   = info.PEs().viewer().name("PEs"),
                Es    = info.PE_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_edge_distance(P, E0, E1, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PE[%d,%d,%d] d^2(%f) out of range, (0,%f)",
                               PE(0),
                               PE(1),
                               PE(2),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PE_barrier_energy(kappa, D_hat, P, E0, E1);
               });

    // Compute Point-Point energy
    auto PP_count = info.PPs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PP_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs   = info.PPs().viewer().name("PPs"),
                Es    = info.PP_energies().viewer().name("Es"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   Float D_hat = d_hat * d_hat;
                   Float D     = D_hat;
                   distance::point_point_distance(P0, P1, D);

                   MUDA_ASSERT(D < D_hat && D > 0,
                               "PP[%d,%d] d^2(%f) out of range, (0,%f)",
                               PP(0),
                               PP(1),
                               D,
                               D_hat);

                   Es(i) = sym::ipc_simplex_contact::PP_barrier_energy(kappa, D_hat, P0, P1);
               });
}

void IPCSimplexNormalContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PTs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs   = info.PTs().viewer().name("PTs"),
                Gs    = info.PT_gradients().viewer().name("Gs"),
                Hs    = info.PT_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   sym::ipc_simplex_contact::PT_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P, T0, T1, T2);
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.EEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs     = info.EEs().viewer().name("EEs"),
                Gs      = info.EE_gradients().viewer().name("Gs"),
                Hs      = info.EE_hessians().viewer().name("Hs"),
                Ps      = info.positions().viewer().name("Ps"),
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   const auto& E0 = Ps(EE[0]);
                   const auto& E1 = Ps(EE[1]);
                   const auto& E2 = Ps(EE[2]);
                   const auto& E3 = Ps(EE[3]);

                   const auto& t0_Ea0 = rest_Ps(EE[0]);
                   const auto& t0_Ea1 = rest_Ps(EE[1]);
                   const auto& t0_Eb0 = rest_Ps(EE[2]);
                   const auto& t0_Eb1 = rest_Ps(EE[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   sym::ipc_simplex_contact::EE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
               });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs   = info.PEs().viewer().name("PEs"),
                Gs    = info.PE_gradients().viewer().name("Gs"),
                Hs    = info.PE_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   const auto& P  = Ps(PE[0]);
                   const auto& E0 = Ps(PE[1]);
                   const auto& E1 = Ps(PE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   sym::ipc_simplex_contact::PE_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P, E0, E1);
               });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.PPs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs   = info.PPs().viewer().name("PPs"),
                Gs    = info.PP_gradients().viewer().name("Gs"),
                Hs    = info.PP_hessians().viewer().name("Hs"),
                Ps    = info.positions().viewer().name("Ps"),
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   const auto& P0 = Ps(PP[0]);
                   const auto& P1 = Ps(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   sym::ipc_simplex_contact::PP_barrier_gradient_hessian(
                       Gs(i), Hs(i), kappa, d_hat * d_hat, P0, P1);
               });
}
}  // namespace uipc::backend::cuda