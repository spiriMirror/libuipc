#pragma once
#include <contact_system/simplex_normal_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/distance/distance_flagged.h>
#include <utils/codim_thickness.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
class IPCSimplexNormalContact final : public SimplexNormalContact
{
  public:
    using SimplexNormalContact::SimplexNormalContact;

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        using namespace muda;
        using namespace sym::codim_ipc_simplex_contact;

        // Compute Point-Triangle energy
        auto PT_count = info.PTs().size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PT_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PTs = info.PTs().viewer().name("PTs"),
                    Es  = info.PT_energies().viewer().name("Es"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       Vector4i PT = PTs(i);


                       auto cid_L = contact_ids(PT[0]);
                       auto cid_R = contact_ids(PT[1]);

                       const auto& P  = Ps(PT[0]);
                       const auto& T0 = Ps(PT[1]);
                       const auto& T1 = Ps(PT[2]);
                       const auto& T2 = Ps(PT[3]);

                       auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                       Float thickness = PT_thickness(thicknesses(PT(0)),
                                                      thicknesses(PT(1)),
                                                      thicknesses(PT(2)),
                                                      thicknesses(PT(3)));

                       Vector4i flag =
                           distance::point_triangle_distance_flag(P, T0, T1, T2);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           Float D_;
                           distance::point_triangle_distance(flag, P, T0, T1, T2, D);
                           distance::point_triangle_distance_unclassified(P, T0, T1, T2, D_);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PT[%d,%d,%d,%d] d^2(%f,%f) out of range, (%f,%f)",
                                       PT(0),
                                       PT(1),
                                       PT(2),
                                       PT(3),
                                       D,
                                       D_,
                                       range(0),
                                       range(1));
                       }

                       Es(i) = PT_barrier_energy(flag, kappa, d_hat, thickness, P, T0, T1, T2);
                   });

        // Compute Edge-Edge energy
        auto EE_count = info.EEs().size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(EE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    EEs = info.EEs().viewer().name("EEs"),
                    Es  = info.EE_energies().viewer().name("Es"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    d_hat   = info.d_hat(),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector4i EE = EEs(i);

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

                       Float thickness = EE_thickness(thicknesses(EE(0)),
                                                      thicknesses(EE(1)),
                                                      thicknesses(EE(2)),
                                                      thicknesses(EE(3)));

                       Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           Float D_;
                           distance::edge_edge_distance(flag, E0, E1, E2, E3, D);
                           distance::edge_edge_distance_unclassified(E0, E1, E2, E3, D_);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "EE[%d,%d,%d,%d] d^2(%f,%f) out of range, (%f,%f), [%d,%d,%d,%d]",
                                       EE(0),
                                       EE(1),
                                       EE(2),
                                       EE(3),
                                       D,
                                       D_,
                                       range(0),
                                       range(1),
                                       flag(0),
                                       flag(1),
                                       flag(2),
                                       flag(3));
                       }

                       Es(i) = mollified_EE_barrier_energy(flag,
                                                           // coefficients
                                                           kappa,
                                                           d_hat,
                                                           thickness,
                                                           // positions
                                                           t0_Ea0,
                                                           t0_Ea1,
                                                           t0_Eb0,
                                                           t0_Eb1,
                                                           E0,
                                                           E1,
                                                           E2,
                                                           E3);
                   });

        // Compute Point-Edge energy
        auto PE_count = info.PEs().size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.PEs().viewer().name("PEs"),
                    Es      = info.PE_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       Vector3i PE = PEs(i);

                       auto cid_L = contact_ids(PE[0]);
                       auto cid_R = contact_ids(PE[1]);

                       const auto& P  = Ps(PE[0]);
                       const auto& E0 = Ps(PE[1]);
                       const auto& E1 = Ps(PE[2]);

                       auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                       Float thickness = PE_thickness(thicknesses(PE(0)),
                                                      thicknesses(PE(1)),
                                                      thicknesses(PE(2)));

                       Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_edge_distance(flag, P, E0, E1, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PE[%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PE(0),
                                       PE(1),
                                       PE(2),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       Es(i) = PE_barrier_energy(flag, kappa, d_hat, thickness, P, E0, E1);
                   });

        // Compute Point-Point energy
        auto PP_count = info.PPs().size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PP_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs     = info.PPs().viewer().name("PPs"),
                    Es      = info.PP_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       Vector2i PP = PPs(i);

                       auto cid_L = contact_ids(PP[0]);
                       auto cid_R = contact_ids(PP[1]);

                       const auto& Pa = Ps(PP[0]);
                       const auto& Pb = Ps(PP[1]);

                       auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                       Float thickness =
                           PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));

                       Vector2i flag = distance::point_point_distance_flag(Pa, Pb);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_point_distance(flag, Pa, Pb, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PP[%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PP(0),
                                       PP(1),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       Es(i) = PP_barrier_energy(flag, kappa, d_hat, thickness, Pa, Pb);
                   });
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace muda;
        using namespace sym::codim_ipc_simplex_contact;

        if(info.PTs().size())
        {
            // Compute Point-Triangle Gradient and Hessian
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(info.PTs().size(),
                       [table = info.contact_tabular().viewer().name("contact_tabular"),
                        contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                        PTs = info.PTs().viewer().name("PTs"),
                        Gs  = info.PT_gradients().viewer().name("Gs"),
                        Hs  = info.PT_hessians().viewer().name("Hs"),
                        Ps  = info.positions().viewer().name("Ps"),
                        thicknesses = info.thicknesses().viewer().name("thicknesses"),
                        d_hat = info.d_hat(),
                        dt    = info.dt()] __device__(int i) mutable
                       {
                           Vector4i PT = PTs(i);

                           auto cid_L = contact_ids(PT[0]);
                           auto cid_R = contact_ids(PT[1]);

                           const auto& P  = Ps(PT[0]);
                           const auto& T0 = Ps(PT[1]);
                           const auto& T1 = Ps(PT[2]);
                           const auto& T2 = Ps(PT[3]);

                           auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                           Float thickness = PT_thickness(thicknesses(PT(0)),
                                                          thicknesses(PT(1)),
                                                          thicknesses(PT(2)),
                                                          thicknesses(PT(3)));

                           Vector4i flag =
                               distance::point_triangle_distance_flag(P, T0, T1, T2);

                           if constexpr(RUNTIME_CHECK)
                           {
                               Float D;
                               distance::point_triangle_distance(flag, P, T0, T1, T2, D);

                               Vector2 range = D_range(thickness, d_hat);

                               MUDA_ASSERT(is_active_D(range, D),
                                           "PT[%d,%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                                           PT(0),
                                           PT(1),
                                           PT(2),
                                           PT(3),
                                           D,
                                           range(0),
                                           range(1));
                           }

                           PT_barrier_gradient_hessian(
                               Gs(i), Hs(i), flag, kappa, d_hat, thickness, P, T0, T1, T2);

                           //cout << "Gs: " << Gs(i).transpose().eval() << "\n"
                           //     << "Hs: " << Hs(i).transpose().eval() << "\n";
                       });
        }


        // Compute Edge-Edge Gradient and Hessian
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.EEs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    EEs = info.EEs().viewer().name("EEs"),
                    Gs  = info.EE_gradients().viewer().name("Gs"),
                    Hs  = info.EE_hessians().viewer().name("Hs"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    d_hat   = info.d_hat(),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector4i EE = EEs(i);


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

                       Float thickness = EE_thickness(thicknesses(EE(0)),
                                                      thicknesses(EE(1)),
                                                      thicknesses(EE(2)),
                                                      thicknesses(EE(3)));

                       Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::edge_edge_distance(flag, E0, E1, E2, E3, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "EE[%d,%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                                       EE(0),
                                       EE(1),
                                       EE(2),
                                       EE(3),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       mollified_EE_barrier_gradient_hessian(
                           Gs(i), Hs(i), flag, kappa, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                   });

        // Compute Point-Edge Gradient and Hessian
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.PEs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.PEs().viewer().name("PEs"),
                    Gs      = info.PE_gradients().viewer().name("Gs"),
                    Hs      = info.PE_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       Vector3i PE = PEs(i);

                       auto cid_L = contact_ids(PE[0]);
                       auto cid_R = contact_ids(PE[1]);

                       const auto& P  = Ps(PE[0]);
                       const auto& E0 = Ps(PE[1]);
                       const auto& E1 = Ps(PE[2]);

                       auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                       Float thickness = PE_thickness(thicknesses(PE(0)),
                                                      thicknesses(PE(1)),
                                                      thicknesses(PE(2)));

                       Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_edge_distance(flag, P, E0, E1, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PE[%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PE(0),
                                       PE(1),
                                       PE(2),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       PE_barrier_gradient_hessian(
                           Gs(i), Hs(i), flag, kappa, d_hat, thickness, P, E0, E1);
                   });

        // Compute Point-Point Gradient and Hessian
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.PPs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs = info.PPs().viewer().name("PPs"),
                    Gs  = info.PP_gradients().viewer().name("Gs"),
                    Hs  = info.PP_hessians().viewer().name("Hs"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PP = PPs(i);

                       auto cid_L = contact_ids(PP[0]);
                       auto cid_R = contact_ids(PP[1]);

                       const auto& P0 = Ps(PP[0]);
                       const auto& P1 = Ps(PP[1]);

                       auto kappa = table(cid_L, cid_R).kappa * dt * dt;

                       Float thickness =
                           PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));

                       Vector2i flag = distance::point_point_distance_flag(P0, P1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_point_distance(flag, P0, P1, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PP[%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PP(0),
                                       PP(1),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       PP_barrier_gradient_hessian(
                           Gs(i), Hs(i), flag, kappa, d_hat, thickness, P0, P1);
                   });
    }
};

REGISTER_SIM_SYSTEM(IPCSimplexNormalContact);
}  // namespace uipc::backend::cuda