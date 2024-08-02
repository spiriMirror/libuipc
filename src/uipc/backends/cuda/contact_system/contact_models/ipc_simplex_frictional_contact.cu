#include <contact_system/contact_models/ipc_simplex_frictional_contact.h>
#include <contact_system/contact_models/ipc_simplex_contact_function.h>

#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(IPCSimplexFrictionalContact);

void IPCSimplexFrictionalContact::do_build(BuildInfo& info) {}

void IPCSimplexFrictionalContact::do_compute_energy(EnergyInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle energy
    auto PT_count = info.friction_PTs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PT_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.friction_PTs().viewer().name("PTs"),
                Es  = info.friction_PT_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);
                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;


                   Float D_hat = d_hat * d_hat;

                   Float          prev_D;
                   const Vector3& prev_P  = prev_Ps(PT[0]);
                   const Vector3& prev_T0 = prev_Ps(PT[1]);
                   const Vector3& prev_T1 = prev_Ps(PT[2]);
                   const Vector3& prev_T2 = prev_Ps(PT[3]);
                   distance::point_triangle_distance(prev_P, prev_T0, prev_T1, prev_T2, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& P  = Ps(PT[0]);
                   const Vector3& T0 = Ps(PT[1]);
                   const Vector3& T1 = Ps(PT[2]);
                   const Vector3& T2 = Ps(PT[3]);
                   Float          D;
                   distance::point_triangle_distance(P, T0, T1, T2, D);
                   // NOTE: D can be larger than D_hat

                   //cout << "PT: " << PT.transpose().eval() << " D/prev_D: " << D
                   //     << "/" << prev_D << "\n";

                   // TODO:
                   Es(i) = 0.0;
               });

    // Compute Edge-Edge energy
    auto EE_count = info.friction_EEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(EE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.friction_EEs().viewer().name("EEs"),
                Es  = info.friction_EE_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   const Vector3& t0_Ea0 = rest_Ps(EE[0]);
                   const Vector3& t0_Ea1 = rest_Ps(EE[1]);
                   const Vector3& t0_Eb0 = rest_Ps(EE[2]);
                   const Vector3& t0_Eb1 = rest_Ps(EE[3]);


                   const Vector3& prev_E0 = prev_Ps(EE[0]);
                   const Vector3& prev_E1 = prev_Ps(EE[1]);
                   const Vector3& prev_E2 = prev_Ps(EE[2]);
                   const Vector3& prev_E3 = prev_Ps(EE[3]);

                   Float D_hat = d_hat * d_hat;

                   Float prev_D;
                   distance::edge_edge_distance(prev_E0, prev_E1, prev_E2, prev_E3, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& E0 = Ps(EE[0]);
                   const Vector3& E1 = Ps(EE[1]);
                   const Vector3& E2 = Ps(EE[2]);
                   const Vector3& E3 = Ps(EE[3]);

                   Float D = D_hat;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);
                   // NOTE: D can be larger than D_hat


                   //cout << "EE: " << EE.transpose().eval() << " D/prev_D: " << D
                   //     << "/" << prev_D << "\n";

                   // TODO:
                   Es(i) = 0.0;
               });

    // Compute Point-Edge energy
    auto PE_count = info.friction_PEs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PE_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.friction_PEs().viewer().name("PEs"),
                Es  = info.friction_PE_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;


                   const Vector3& prev_P  = prev_Ps(PE[0]);
                   const Vector3& prev_E0 = prev_Ps(PE[1]);
                   const Vector3& prev_E1 = prev_Ps(PE[2]);
                   Float          prev_D;
                   distance::point_edge_distance(prev_P, prev_E0, prev_E1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P  = Ps(PE[0]);
                   const Vector3& E0 = Ps(PE[1]);
                   const Vector3& E1 = Ps(PE[2]);
                   Float          D;
                   distance::point_edge_distance(P, E0, E1, D);
                   // NOTE: D can be larger than D_hat

                   //cout << "PE: " << PE.transpose().eval() << " D/prev_D: " << D
                   //     << "/" << prev_D << "\n";

                   // TODO:
                   Es(i) = 0.0;
               });

    // Compute Point-Point energy
    auto PP_count = info.friction_PPs().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(PP_count,
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.friction_PPs().viewer().name("PPs"),
                Es  = info.friction_PP_energies().viewer().name("Es"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P0 = prev_Ps(PP[0]);
                   const Vector3& prev_P1 = prev_Ps(PP[1]);
                   Float          prev_D;
                   distance::point_point_distance(prev_P0, prev_P1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& P0 = Ps(PP[0]);
                   const Vector3& P1 = Ps(PP[1]);
                   Float          D  = D_hat;
                   distance::point_point_distance(P0, P1, D);
                   // NOTE: D can be larger than D_hat

                   //cout << "PP: " << PP.transpose().eval() << " D/prev_D: " << D
                   //     << "/" << prev_D << "\n";

                   // TODO:
                   Es(i) = 0.0;
               });
}

void IPCSimplexFrictionalContact::do_assemble(ContactInfo& info)
{
    using namespace muda;

    // Compute Point-Triangle Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PTs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PTs = info.friction_PTs().viewer().name("PTs"),
                Gs  = info.friction_PT_gradients().viewer().name("Gs"),
                Hs  = info.friction_PT_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PT = PTs(i);

                   auto cid_L = contact_ids(PT[0]);
                   auto cid_R = contact_ids(PT[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P  = prev_Ps(PT[0]);
                   const Vector3& prev_T0 = prev_Ps(PT[1]);
                   const Vector3& prev_T1 = prev_Ps(PT[2]);
                   const Vector3& prev_T2 = prev_Ps(PT[3]);

                   const Vector3& P  = Ps(PT[0]);
                   const Vector3& T0 = Ps(PT[1]);
                   const Vector3& T1 = Ps(PT[2]);
                   const Vector3& T2 = Ps(PT[3]);

                   Float D;
                   distance::point_triangle_distance(P, T0, T1, T2, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction


                   // TODO:
                   Gs(i).setZero();
                   Hs(i).setZero();
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_EEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                EEs = info.friction_EEs().viewer().name("EEs"),
                Gs  = info.friction_EE_gradients().viewer().name("Gs"),
                Hs  = info.friction_EE_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v   = info.eps_velocity(),
                rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                d_hat   = info.d_hat(),
                dt      = info.dt()] __device__(int i) mutable
               {
                   const auto& EE = EEs(i);

                   auto cid_L = contact_ids(EE[0]);
                   auto cid_R = contact_ids(EE[2]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   const Vector3& t0_Ea0 = rest_Ps(EE[0]);
                   const Vector3& t0_Ea1 = rest_Ps(EE[1]);
                   const Vector3& t0_Eb0 = rest_Ps(EE[2]);
                   const Vector3& t0_Eb1 = rest_Ps(EE[3]);

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_E0 = prev_Ps(EE[0]);
                   const Vector3& prev_E1 = prev_Ps(EE[1]);
                   const Vector3& prev_E2 = prev_Ps(EE[2]);
                   const Vector3& prev_E3 = prev_Ps(EE[3]);

                   Float prev_D;
                   distance::edge_edge_distance(prev_E0, prev_E1, prev_E2, prev_E3, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);


                   const Vector3& E0 = Ps(EE[0]);
                   const Vector3& E1 = Ps(EE[1]);
                   const Vector3& E2 = Ps(EE[2]);
                   const Vector3& E3 = Ps(EE[3]);

                   Float D;
                   distance::edge_edge_distance(E0, E1, E2, E3, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction

                   // TODO:
                   Gs(i).setZero();
                   Hs(i).setZero();
               });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PEs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PEs = info.friction_PEs().viewer().name("PEs"),
                Gs  = info.friction_PE_gradients().viewer().name("Gs"),
                Hs  = info.friction_PE_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PE = PEs(i);

                   auto cid_L = contact_ids(PE[0]);
                   auto cid_R = contact_ids(PE[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P  = prev_Ps(PE[0]);
                   const Vector3& prev_E0 = prev_Ps(PE[1]);
                   const Vector3& prev_E1 = prev_Ps(PE[2]);
                   Float          prev_D;
                   distance::point_edge_distance(prev_P, prev_E0, prev_E1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P  = Ps(PE[0]);
                   const Vector3& E0 = Ps(PE[1]);
                   const Vector3& E1 = Ps(PE[2]);
                   Float          D;
                   distance::point_edge_distance(P, E0, E1, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction

                   Gs(i).setZero();
                   Hs(i).setZero();
               });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.friction_PPs().size(),
               [table = info.contact_tabular().viewer().name("contact_tabular"),
                contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                PPs = info.friction_PPs().viewer().name("PPs"),
                Gs  = info.friction_PP_gradients().viewer().name("Gs"),
                Hs  = info.friction_PP_hessians().viewer().name("Hs"),
                Ps  = info.positions().viewer().name("Ps"),
                prev_Ps = info.prev_positions().viewer().name("prev_Ps"),  // for friction calculation
                eps_v = info.eps_velocity(),
                d_hat = info.d_hat(),
                dt    = info.dt()] __device__(int i) mutable
               {
                   const auto& PP = PPs(i);

                   auto cid_L = contact_ids(PP[0]);
                   auto cid_R = contact_ids(PP[1]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;
                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Float D_hat = d_hat * d_hat;

                   const Vector3& prev_P0 = prev_Ps(PP[0]);
                   const Vector3& prev_P1 = prev_Ps(PP[1]);
                   Float          prev_D;
                   distance::point_point_distance(prev_P0, prev_P1, prev_D);
                   MUDA_ASSERT(prev_D < D_hat, "prev_D(%f) out of range, (0,%f)", prev_D, D_hat);

                   const Vector3& P0 = Ps(PP[0]);
                   const Vector3& P1 = Ps(PP[1]);
                   Float          D  = D_hat;
                   distance::point_point_distance(P0, P1, D);
                   // NOTE: D can be larger than D_hat, if so, ignore this friction

                   Gs(i).setZero();
                   Hs(i).setZero();
               });
}
}  // namespace uipc::backend::cuda