#include <contact_system/contact_models/ipc_simplex_frictional_contact.h>
#include <contact_system/contact_models/ipc_simplex_frictional_contact_function.h>

#include <kernel_cout.h>

namespace uipc::backend::cuda
{
constexpr bool DebugTest = false;

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

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

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

                   Float D;
                   distance::point_triangle_distance(P, T0, T1, T2, D);
                   // NOTE: D can be larger than D_hat

                   // cout << "PT: " << PT.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PT_friction_energy(
                       kappa, D_hat, friction_rate, dt, P, T0, T1, T2, prev_P, prev_T0, prev_T1, prev_T2, eps_v);
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

                   // cout << "EE: " << EE.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::EE_friction_energy(
                       kappa, D_hat, friction_rate, dt, E0, E1, E2, E3, prev_E0, prev_E1, prev_E2, prev_E3, eps_v);
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

                   // cout << "PE: " << PE.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PE_friction_energy(
                       kappa, D_hat, friction_rate, dt, P, E0, E1, prev_P, prev_E0, prev_E1, eps_v);
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

                   // cout << "PP: " << PP.transpose().eval() << " D/prev_D: " << D
                   //      << "/" << prev_D << "\n";

                   Es(i) = sym::ipc_simplex_contact::PP_friction_energy(
                       kappa, D_hat, friction_rate, dt, P0, P1, prev_P0, prev_P1, eps_v);
               });
}

namespace sym::ipc_simplex_contact
{
    __device__ void PT_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector4i& PT,
                                     const Vector3&  P,
                                     const Vector3&  T0,
                                     const Vector3&  T1,
                                     const Vector3&  T2,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector12&               G_friction,
                                     const Matrix12x12&            H_friction);

    __device__ void EE_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector4i& EE,
                                     const Vector3&  E0,
                                     const Vector3&  E1,
                                     const Vector3&  E2,
                                     const Vector3&  E3,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector12&                G_friction,
                                     const Matrix12x12&             H_friction);

    __device__ void PE_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector3i& PE,
                                     const Vector3&  P,
                                     const Vector3&  E0,
                                     const Vector3&  E1,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector9&                 G_friction,
                                     const Matrix9x9&               H_friction);

    __device__ void PP_friction_test(Float            kappa,
                                     Float            d_hat,
                                     Float            friction_rate,
                                     Float            dt,
                                     Float            eps_v,
                                     const Vector2i&  PP,
                                     const Vector3&   P0,
                                     const Vector3&   P1,
                                     const Vector3&   prev_P0,
                                     const Vector3&   prev_P1,
                                     const Vector6&   G_friction,
                                     const Matrix6x6& H_friction);
}  // namespace sym::ipc_simplex_contact

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

                   const auto& P  = Ps(PT[0]);
                   const auto& T0 = Ps(PT[1]);
                   const auto& T1 = Ps(PT[2]);
                   const auto& T2 = Ps(PT[3]);

                   auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                   // Use this to compute friction
                   auto friction_rate = table(cid_L, cid_R).mu;

                   Vector12    G_friction;
                   Matrix12x12 H_friction;
                   sym::ipc_simplex_contact::PT_friction_gradient_hessian(
                       G_friction,
                       H_friction,
                       kappa,
                       d_hat * d_hat,
                       friction_rate,
                       dt,
                       P,
                       T0,
                       T1,
                       T2,
                       prev_Ps(PT[0]),
                       prev_Ps(PT[1]),
                       prev_Ps(PT[2]),
                       prev_Ps(PT[3]),
                       eps_v);


                   if constexpr(DebugTest)
                   {
                       // Gradient check
                       sym::ipc_simplex_contact::PT_friction_test(
                           kappa, d_hat, friction_rate, dt, eps_v, PT, P, T0, T1, T2, prev_Ps, G_friction, H_friction);
                   }

                   Gs(i) = G_friction;
                   Hs(i) = H_friction;
               });

    // Compute Edge-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(
            info.friction_EEs().size(),
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

                Vector12    G_friction;
                Matrix12x12 H_friction;
                sym::ipc_simplex_contact::EE_friction_gradient_hessian(G_friction,
                                                                       H_friction,
                                                                       kappa,
                                                                       d_hat * d_hat,
                                                                       friction_rate,
                                                                       dt,
                                                                       E0,
                                                                       E1,
                                                                       E2,
                                                                       E3,
                                                                       prev_E0,
                                                                       prev_E1,
                                                                       prev_E2,
                                                                       prev_E3,
                                                                       eps_v);

                if constexpr(DebugTest)
                {
                    // Gradient check
                    sym::ipc_simplex_contact::EE_friction_test(
                        kappa, d_hat, friction_rate, dt, eps_v, EE, E0, E1, E2, E3, prev_Ps, G_friction, H_friction);
                }

                Gs(i) = G_friction;
                Hs(i) = H_friction;
            });

    // Compute Point-Edge Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(
            info.friction_PEs().size(),
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

                Vector9   G_friction = Eigen::Matrix<Float, 9, 1>::Zero();
                Matrix9x9 H_friction = Eigen::Matrix<Float, 9, 9>::Zero();
                sym::ipc_simplex_contact::PE_friction_gradient_hessian(
                    G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P, E0, E1, prev_P, prev_E0, prev_E1, eps_v);

                if constexpr(DebugTest)
                {
                    // Gradient check
                    sym::ipc_simplex_contact::PE_friction_test(
                        kappa, d_hat, friction_rate, dt, eps_v, PE, P, E0, E1, prev_Ps, G_friction, H_friction);
                }

                Gs(i) = G_friction;
                Hs(i) = H_friction;
            });

    // Compute Point-Point Gradient and Hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(
            info.friction_PPs().size(),
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
                Vector6   G_friction;
                Matrix6x6 H_friction;
                sym::ipc_simplex_contact::PP_friction_gradient_hessian(
                    G_friction, H_friction, kappa, d_hat * d_hat, friction_rate, dt, P0, P1, prev_P0, prev_P1, eps_v);


                if constexpr(DebugTest)
                {
                    // Gradient check
                    sym::ipc_simplex_contact::PP_friction_test(
                        kappa, d_hat, friction_rate, dt, eps_v, PP, P0, P1, prev_P0, prev_P1, G_friction, H_friction);
                }


                Gs(i) = G_friction;
                Hs(i) = H_friction;
            });
}

namespace sym::ipc_simplex_contact
{
    __device__ void PT_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector4i& PT,
                                     const Vector3&  P,
                                     const Vector3&  T0,
                                     const Vector3&  T1,
                                     const Vector3&  T2,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector12&                G_friction,
                                     const Matrix12x12&             H_friction)
    {
        Vector3 test;
        test(0)  = 1e-8;
        test(1)  = 1e-8;
        test(2)  = 1e-8;
        Float E1 = sym::ipc_simplex_contact::PT_friction_energy(kappa,
                                                                d_hat * d_hat,
                                                                friction_rate,
                                                                dt,
                                                                P + test,
                                                                T0,
                                                                T1,
                                                                T2,
                                                                prev_Ps(PT[0]),
                                                                prev_Ps(PT[1]),
                                                                prev_Ps(PT[2]),
                                                                prev_Ps(PT[3]),
                                                                eps_v);
        Float E2 = sym::ipc_simplex_contact::PT_friction_energy(kappa,
                                                                d_hat * d_hat,
                                                                friction_rate,
                                                                dt,
                                                                P - test,
                                                                T0,
                                                                T1,
                                                                T2,
                                                                prev_Ps(PT[0]),
                                                                prev_Ps(PT[1]),
                                                                prev_Ps(PT[2]),
                                                                prev_Ps(PT[3]),
                                                                eps_v);
        Float numerical_diff = (E1 - E2) / 2;
        cout << "numerical_diff: " << numerical_diff << "\n";
        Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1)
                                + G_friction(2) * test(2);
        cout << "analytical_diff: " << analytical_diff << "\n";
        cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff
             << "\n";
        cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff
             << "\n";

        Vector12    G_friction1 = Eigen::Matrix<Float, 12, 1>::Zero();
        Vector12    G_friction2 = Eigen::Matrix<Float, 12, 1>::Zero();
        Matrix12x12 H_friction0 = Eigen::Matrix<Float, 12, 12>::Zero();
        Vector12    test12      = Eigen::Matrix<Float, 12, 1>::Zero();
        test12(0)               = 1e-8;
        test12(1)               = 1e-8;
        test12(2)               = 1e-8;
        sym::ipc_simplex_contact::PT_friction_gradient_hessian(G_friction1,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P + test12.segment<3>(0),
                                                               T0,
                                                               T1,
                                                               T2,
                                                               prev_Ps(PT[0]),
                                                               prev_Ps(PT[1]),
                                                               prev_Ps(PT[2]),
                                                               prev_Ps(PT[3]),
                                                               eps_v);
        sym::ipc_simplex_contact::PT_friction_gradient_hessian(G_friction2,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P - test12.segment<3>(0),
                                                               T0,
                                                               T1,
                                                               T2,
                                                               prev_Ps(PT[0]),
                                                               prev_Ps(PT[1]),
                                                               prev_Ps(PT[2]),
                                                               prev_Ps(PT[3]),
                                                               eps_v);

        Vector12 G_friction_numerical_diff  = (G_friction1 - G_friction2) / 2;
        Vector12 G_friction_analytical_diff = H_friction * test12;
        cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
        cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
        cout << "PT_grad_diff: "
             << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
    }

    __device__ void EE_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector4i& EE,
                                     const Vector3&  E0,
                                     const Vector3&  E1,
                                     const Vector3&  E2,
                                     const Vector3&  E3,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector12&                G_friction,
                                     const Matrix12x12&             H_friction)
    {
        Vector3 test;
        test(0)   = 1e-6;
        test(1)   = 1e-6;
        test(2)   = 1e-6;
        Float E1_ = sym::ipc_simplex_contact::EE_friction_energy(kappa,
                                                                 d_hat * d_hat,
                                                                 friction_rate,
                                                                 dt,
                                                                 E0 + test,
                                                                 E1,
                                                                 E2,
                                                                 E3,
                                                                 prev_Ps(EE[0]),
                                                                 prev_Ps(EE[1]),
                                                                 prev_Ps(EE[2]),
                                                                 prev_Ps(EE[3]),
                                                                 eps_v);
        Float E2_ = sym::ipc_simplex_contact::EE_friction_energy(kappa,
                                                                 d_hat * d_hat,
                                                                 friction_rate,
                                                                 dt,
                                                                 E0 - test,
                                                                 E1,
                                                                 E2,
                                                                 E3,
                                                                 prev_Ps(EE[0]),
                                                                 prev_Ps(EE[1]),
                                                                 prev_Ps(EE[2]),
                                                                 prev_Ps(EE[3]),
                                                                 eps_v);
        Float numerical_diff = (E1_ - E2_) / 2;
        cout << "numerical_diff: " << numerical_diff << "\n";
        Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1)
                                + G_friction(2) * test(2);
        cout << "analytical_diff: " << analytical_diff << "\n";
        cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff
             << "\n";
        cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff
             << "\n";

        Vector12    G_friction1 = Eigen::Matrix<Float, 12, 1>::Zero();
        Vector12    G_friction2 = Eigen::Matrix<Float, 12, 1>::Zero();
        Matrix12x12 H_friction0 = Eigen::Matrix<Float, 12, 12>::Zero();
        Vector12    test12      = Eigen::Matrix<Float, 12, 1>::Zero();
        test12(0)               = 1e-8;
        test12(1)               = 1e-8;
        test12(2)               = 1e-8;
        sym::ipc_simplex_contact::EE_friction_gradient_hessian(G_friction1,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               E0 + test12.segment<3>(0),
                                                               E1,
                                                               E2,
                                                               E3,
                                                               prev_Ps(EE[0]),
                                                               prev_Ps(EE[1]),
                                                               prev_Ps(EE[2]),
                                                               prev_Ps(EE[3]),
                                                               eps_v);
        sym::ipc_simplex_contact::EE_friction_gradient_hessian(G_friction2,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               E0 - test12.segment<3>(0),
                                                               E1,
                                                               E2,
                                                               E3,
                                                               prev_Ps(EE[0]),
                                                               prev_Ps(EE[1]),
                                                               prev_Ps(EE[2]),
                                                               prev_Ps(EE[3]),
                                                               eps_v);

        Vector12 G_friction_numerical_diff  = (G_friction1 - G_friction2) / 2;
        Vector12 G_friction_analytical_diff = H_friction * test12;
        cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
        cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
        cout << "EE_grad_diff: "
             << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
    }

    __device__ void PE_friction_test(Float           kappa,
                                     Float           d_hat,
                                     Float           friction_rate,
                                     Float           dt,
                                     Float           eps_v,
                                     const Vector3i& PE,
                                     const Vector3&  P,
                                     const Vector3&  E0,
                                     const Vector3&  E1,
                                     const muda::CDense1D<Vector3>& prev_Ps,
                                     const Vector9&                 G_friction,
                                     const Matrix9x9&               H_friction)
    {
        Vector3 test;
        test(0)   = 1e-6;
        test(1)   = 1e-6;
        test(2)   = 1e-6;
        Float E1_ = sym::ipc_simplex_contact::PE_friction_energy(kappa,
                                                                 d_hat * d_hat,
                                                                 friction_rate,
                                                                 dt,
                                                                 P + test,
                                                                 E0,
                                                                 E1,
                                                                 prev_Ps(PE[0]),
                                                                 prev_Ps(PE[1]),
                                                                 prev_Ps(PE[2]),
                                                                 eps_v);
        Float E2_ = sym::ipc_simplex_contact::PE_friction_energy(kappa,
                                                                 d_hat * d_hat,
                                                                 friction_rate,
                                                                 dt,
                                                                 P - test,
                                                                 E0,
                                                                 E1,
                                                                 prev_Ps(PE[0]),
                                                                 prev_Ps(PE[1]),
                                                                 prev_Ps(PE[2]),
                                                                 eps_v);
        Float numerical_diff = (E1_ - E2_) / 2;
        cout << "numerical_diff: " << numerical_diff << "\n";
        Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1)
                                + G_friction(2) * test(2);
        cout << "analytical_diff: " << analytical_diff << "\n";
        cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff
             << "\n";
        cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff
             << "\n";

        Vector9   G_friction1 = Eigen::Matrix<Float, 9, 1>::Zero();
        Vector9   G_friction2 = Eigen::Matrix<Float, 9, 1>::Zero();
        Matrix9x9 H_friction0 = Eigen::Matrix<Float, 9, 9>::Zero();
        Vector9   test9       = Eigen::Matrix<Float, 9, 1>::Zero();
        test9(0)              = 1e-8;
        test9(1)              = 1e-8;
        test9(2)              = 1e-8;
        sym::ipc_simplex_contact::PE_friction_gradient_hessian(G_friction1,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P + test9.segment<3>(0),
                                                               E0,
                                                               E1,
                                                               prev_Ps(PE[0]),
                                                               prev_Ps(PE[1]),
                                                               prev_Ps(PE[2]),
                                                               eps_v);
        sym::ipc_simplex_contact::PE_friction_gradient_hessian(G_friction2,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P - test9.segment<3>(0),
                                                               E0,
                                                               E1,
                                                               prev_Ps(PE[0]),
                                                               prev_Ps(PE[1]),
                                                               prev_Ps(PE[2]),
                                                               eps_v);

        Vector9 G_friction_numerical_diff  = (G_friction1 - G_friction2) / 2;
        Vector9 G_friction_analytical_diff = H_friction * test9;
        cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
        cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
        cout << "E1_: " << E1_ << "\n";
        cout << "G_friction: " << G_friction.norm() << "\n";
        cout << "H_friction: " << H_friction.norm() << "\n";
        cout << "PE_grad_diff: "
             << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
    }

    __device__ void PP_friction_test(Float            kappa,
                                     Float            d_hat,
                                     Float            friction_rate,
                                     Float            dt,
                                     Float            eps_v,
                                     const Vector2i&  PP,
                                     const Vector3&   P0,
                                     const Vector3&   P1,
                                     const Vector3&   prev_P0,
                                     const Vector3&   prev_P1,
                                     const Vector6&   G_friction,
                                     const Matrix6x6& H_friction)
    {
        Vector3 test;
        test(0)   = 1e-6;
        test(1)   = 1e-6;
        test(2)   = 1e-6;
        Float E1_ = sym::ipc_simplex_contact::PP_friction_energy(
            kappa, d_hat * d_hat, friction_rate, dt, P0 + test, P1, prev_P0, prev_P1, eps_v);
        Float E2_ = sym::ipc_simplex_contact::PP_friction_energy(
            kappa, d_hat * d_hat, friction_rate, dt, P0 - test, P1, prev_P0, prev_P1, eps_v);
        Float numerical_diff = (E1_ - E2_) / 2;
        cout << "numerical_diff: " << numerical_diff << "\n";
        Float analytical_diff = G_friction(0) * test(0) + G_friction(1) * test(1)
                                + G_friction(2) * test(2);
        cout << "analytical_diff: " << analytical_diff << "\n";
        cout << "numerical_diff - analytical_diff: " << numerical_diff - analytical_diff
             << "\n";
        cout << "numerical_diff / analytical_diff: " << numerical_diff / analytical_diff
             << "\n";

        Vector6   G_friction1 = Eigen::Matrix<Float, 6, 1>::Zero();
        Vector6   G_friction2 = Eigen::Matrix<Float, 6, 1>::Zero();
        Matrix6x6 H_friction0 = Eigen::Matrix<Float, 6, 6>::Zero();
        Vector6   test6       = Eigen::Matrix<Float, 6, 1>::Zero();
        test6(0)              = 1e-8;
        test6(1)              = 1e-8;
        test6(2)              = 1e-8;
        sym::ipc_simplex_contact::PP_friction_gradient_hessian(G_friction1,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P0 + test6.segment<3>(0),
                                                               P1,
                                                               prev_P0,
                                                               prev_P1,
                                                               eps_v);
        sym::ipc_simplex_contact::PP_friction_gradient_hessian(G_friction2,
                                                               H_friction0,
                                                               kappa,
                                                               d_hat * d_hat,
                                                               friction_rate,
                                                               dt,
                                                               P0 - test6.segment<3>(0),
                                                               P1,
                                                               prev_P0,
                                                               prev_P1,
                                                               eps_v);

        Vector6 G_friction_numerical_diff  = (G_friction1 - G_friction2) / 2;
        Vector6 G_friction_analytical_diff = H_friction * test6;
        cout << "G_friction_numerical_diff: " << G_friction_numerical_diff.norm() << "\n";
        cout << "G_friction_analytical_diff: " << G_friction_analytical_diff.norm() << "\n";
        cout << "EE_grad_diff: "
             << (G_friction_numerical_diff - G_friction_analytical_diff).norm() << "\n";
    }
}  // namespace sym::ipc_simplex_contact
}  // namespace uipc::backend::cuda