#include <contact_system/simplex_normal_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/distance/distance_flagged.h>
#include <utils/codim_thickness.h>
#include <kernel_cout.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>
#include <utils/primitive_d_hat.h>

namespace uipc::backend::cuda
{
class IPCSimplexNormalContact final : public SimplexNormalContact
{
  public:
    using SimplexNormalContact::SimplexNormalContact;

    virtual void do_build(BuildInfo& info) override
    {
        auto constitution =
            world().scene().config().find<std::string>("contact/constitution");
        if(constitution->view()[0] != "ipc")
        {
            throw SimSystemException("Constitution is not IPC");
        }
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        using namespace muda;
        using namespace sym::codim_ipc_simplex_contact;

        // Compute Point-Triangle energy
        auto PT_count = info.PTs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(PT_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PTs = info.PTs().viewer().name("PTs"),
                    Es  = info.PT_energies().viewer().name("Es"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
                       Vector4i PT = PTs(i);

                       Vector4i cids = {contact_ids(PT[0]),
                                        contact_ids(PT[1]),
                                        contact_ids(PT[2]),
                                        contact_ids(PT[3])};
                       Float    kt2  = PT_kappa(table, cids) * dt * dt;

                       const auto& P  = Ps(PT[0]);
                       const auto& T0 = Ps(PT[1]);
                       const auto& T1 = Ps(PT[2]);
                       const auto& T2 = Ps(PT[3]);


                       Float thickness = PT_thickness(thicknesses(PT(0)),
                                                      thicknesses(PT(1)),
                                                      thicknesses(PT(2)),
                                                      thicknesses(PT(3)));

                       Float d_hat = PT_d_hat(
                           d_hats(PT(0)), d_hats(PT(1)), d_hats(PT(2)), d_hats(PT(3)));

                       Vector4i flag =
                           distance::point_triangle_distance_flag(P, T0, T1, T2);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

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

                       Es(i) = PT_barrier_energy(flag, kt2, d_hat, thickness, P, T0, T1, T2);
                   });

        // Compute Edge-Edge energy
        auto EE_count = info.EEs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(EE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    EEs = info.EEs().viewer().name("EEs"),
                    Es  = info.EE_energies().viewer().name("Es"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    d_hats  = info.d_hats().viewer().name("d_hats"),
                    dt      = info.dt()] __device__(int i) mutable
                   {
                       Vector4i EE = EEs(i);

                       Vector4i cids = {contact_ids(EE[0]),
                                        contact_ids(EE[1]),
                                        contact_ids(EE[2]),
                                        contact_ids(EE[3])};
                       Float    kt2  = EE_kappa(table, cids) * dt * dt;

                       const auto& E0 = Ps(EE[0]);
                       const auto& E1 = Ps(EE[1]);
                       const auto& E2 = Ps(EE[2]);
                       const auto& E3 = Ps(EE[3]);

                       const auto& t0_Ea0 = rest_Ps(EE[0]);
                       const auto& t0_Ea1 = rest_Ps(EE[1]);
                       const auto& t0_Eb0 = rest_Ps(EE[2]);
                       const auto& t0_Eb1 = rest_Ps(EE[3]);

                       Float thickness = EE_thickness(thicknesses(EE(0)),
                                                      thicknesses(EE(1)),
                                                      thicknesses(EE(2)),
                                                      thicknesses(EE(3)));

                       Float d_hat = EE_d_hat(
                           d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));

                       Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);
                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "EE[%d,%d,%d,%d] d^2(%f) out of range, (%f,%f), [%d,%d,%d,%d]",
                                       EE(0),
                                       EE(1),
                                       EE(2),
                                       EE(3),
                                       D,
                                       range(0),
                                       range(1),
                                       flag(0),
                                       flag(1),
                                       flag(2),
                                       flag(3));
                       }


                       Es(i) = mollified_EE_barrier_energy(flag,
                                                           // coefficients
                                                           kt2,
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
            .file_line(__FILE__, __LINE__)
            .apply(PE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.PEs().viewer().name("PEs"),
                    Es      = info.PE_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v  = info.eps_velocity(),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
                       Vector3i PE = PEs(i);

                       Vector3i cids = {contact_ids(PE[0]),
                                        contact_ids(PE[1]),
                                        contact_ids(PE[2])};
                       Float    kt2  = PE_kappa(table, cids) * dt * dt;

                       const auto& P  = Ps(PE[0]);
                       const auto& E0 = Ps(PE[1]);
                       const auto& E1 = Ps(PE[2]);

                       Float thickness = PE_thickness(thicknesses(PE(0)),
                                                      thicknesses(PE(1)),
                                                      thicknesses(PE(2)));

                       Float d_hat =
                           PE_d_hat(d_hats(PE(0)), d_hats(PE(1)), d_hats(PE(2)));

                       Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_edge_distance2(flag, P, E0, E1, D);

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

                       Es(i) = PE_barrier_energy(flag, kt2, d_hat, thickness, P, E0, E1);
                   });

        // Compute Point-Point energy
        auto PP_count = info.PPs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(PP_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs     = info.PPs().viewer().name("PPs"),
                    Es      = info.PP_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
                       Vector2i PP = PPs(i);

                       Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
                       Float    kt2  = PP_kappa(table, cids) * dt * dt;

                       const auto& Pa = Ps(PP[0]);
                       const auto& Pb = Ps(PP[1]);

                       Float thickness =
                           PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));

                       Float d_hat = PP_d_hat(d_hats(PP(0)), d_hats(PP(1)));

                       Vector2i flag = distance::point_point_distance_flag(Pa, Pb);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_point_distance2(flag, Pa, Pb, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PP[%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PP(0),
                                       PP(1),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       Es(i) = PP_barrier_energy(flag, kt2, d_hat, thickness, Pa, Pb);
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
                .file_line(__FILE__, __LINE__)
                .apply(
                    info.PTs().size(),
                    [gradient_only = info.gradient_only(),
                     table = info.contact_tabular().viewer().name("contact_tabular"),
                     contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                     PTs = info.PTs().viewer().name("PTs"),
                     Gs  = info.PT_gradients().viewer().name("Gs"),
                     Hs  = info.PT_hessians().viewer().name("Hs"),
                     Ps  = info.positions().viewer().name("Ps"),
                     thicknesses = info.thicknesses().viewer().name("thicknesses"),
                     d_hats = info.d_hats().viewer().name("d_hats"),
                     dt     = info.dt(),
                     cout   = KernelCout::viewer()] __device__(int i) mutable
                    {
                        Vector4i PT = PTs(i);

                        Vector4i cids = {contact_ids(PT[0]),
                                         contact_ids(PT[1]),
                                         contact_ids(PT[2]),
                                         contact_ids(PT[3])};
                        Float    kt2  = PT_kappa(table, cids) * dt * dt;


                        const auto& P  = Ps(PT[0]);
                        const auto& T0 = Ps(PT[1]);
                        const auto& T1 = Ps(PT[2]);
                        const auto& T2 = Ps(PT[3]);


                        Float thickness = PT_thickness(thicknesses(PT(0)),
                                                       thicknesses(PT(1)),
                                                       thicknesses(PT(2)),
                                                       thicknesses(PT(3)));

                        Float d_hat = PT_d_hat(
                            d_hats(PT(0)), d_hats(PT(1)), d_hats(PT(2)), d_hats(PT(3)));

                        Vector4i flag =
                            distance::point_triangle_distance_flag(P, T0, T1, T2);

                        if constexpr(RUNTIME_CHECK)
                        {
                            Float D;
                            distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

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

                        Vector12 G;
                        if(gradient_only)
                        {
                            Float D;
                            distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

                            Matrix12x12 H;
                            PT_barrier_gradient_hessian(
                                G, H, flag, kt2, d_hat, thickness, P, T0, T1, T2);

                            Vector12 new_G;
                            PT_barrier_gradient(
                                new_G, flag, kt2, d_hat, thickness, P, T0, T1, T2);


                            // PT_barrier_gradient(G, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                            DoubletVectorAssembler DVA{Gs};
                            DVA.segment<4>(i * 4).write(PT, G);

                            //cout << "G:" << G.transpose().eval() << "\n"
                            //     << "d:" << sqrt(D) << "\n"
                            //     << "new_G:" << new_G.transpose().eval() << "\n";
                        }
                        else
                        {
                            Matrix12x12 H;
                            PT_barrier_gradient_hessian(
                                G, H, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                            make_spd(H);
                            DoubletVectorAssembler DVA{Gs};
                            DVA.segment<4>(i * 4).write(PT, G);
                            TripletMatrixAssembler TMA{Hs};
                            TMA.half_block<4>(i * PTHalfHessianSize).write(PT, H);
                        }
                    });
        }


        // Compute Edge-Edge Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                info.EEs().size(),
                [gradient_only = info.gradient_only(),
                 table = info.contact_tabular().viewer().name("contact_tabular"),
                 contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                 EEs         = info.EEs().viewer().name("EEs"),
                 Gs          = info.EE_gradients().viewer().name("Gs"),
                 Hs          = info.EE_hessians().viewer().name("Hs"),
                 Ps          = info.positions().viewer().name("Ps"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 rest_Ps     = info.rest_positions().viewer().name("rest_Ps"),
                 d_hats      = info.d_hats().viewer().name("d_hats"),
                 dt          = info.dt()] __device__(int i) mutable
                {
                    Vector4i EE = EEs(i);

                    Vector4i cids = {contact_ids(EE[0]),
                                     contact_ids(EE[1]),
                                     contact_ids(EE[2]),
                                     contact_ids(EE[3])};
                    Float    kt2  = EE_kappa(table, cids) * dt * dt;

                    const auto& E0 = Ps(EE[0]);
                    const auto& E1 = Ps(EE[1]);
                    const auto& E2 = Ps(EE[2]);
                    const auto& E3 = Ps(EE[3]);

                    const auto& t0_Ea0 = rest_Ps(EE[0]);
                    const auto& t0_Ea1 = rest_Ps(EE[1]);
                    const auto& t0_Eb0 = rest_Ps(EE[2]);
                    const auto& t0_Eb1 = rest_Ps(EE[3]);

                    Float thickness = EE_thickness(thicknesses(EE(0)),
                                                   thicknesses(EE(1)),
                                                   thicknesses(EE(2)),
                                                   thicknesses(EE(3)));

                    Float d_hat = EE_d_hat(
                        d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));

                    Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

                    if constexpr(RUNTIME_CHECK)
                    {
                        Float D;
                        distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);

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

                    Vector12 G;
                    if(gradient_only)
                    {
                        mollified_EE_barrier_gradient(
                            G, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                        DoubletVectorAssembler DVA{Gs};
                        DVA.segment<4>(i * 4).write(EE, G);
                    }
                    else
                    {
                        Matrix12x12 H;
                        mollified_EE_barrier_gradient_hessian(
                            G, H, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);

                        make_spd(H);
                        DoubletVectorAssembler DVA{Gs};
                        DVA.segment<4>(i * 4).write(EE, G);
                        TripletMatrixAssembler TMA{Hs};
                        TMA.half_block<4>(i * EEHalfHessianSize).write(EE, H);
                    }
                });

        // Compute Point-Edge Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.PEs().size(),
                   [gradient_only = info.gradient_only(),
                    table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.PEs().viewer().name("PEs"),
                    Gs      = info.PE_gradients().viewer().name("Gs"),
                    Hs      = info.PE_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
                       Vector3i PE = PEs(i);

                       Vector3i cids = {contact_ids(PE[0]),
                                        contact_ids(PE[1]),
                                        contact_ids(PE[2])};
                       Float    kt2  = PE_kappa(table, cids) * dt * dt;

                       const auto& P  = Ps(PE[0]);
                       const auto& E0 = Ps(PE[1]);
                       const auto& E1 = Ps(PE[2]);

                       Float thickness = PE_thickness(thicknesses(PE(0)),
                                                      thicknesses(PE(1)),
                                                      thicknesses(PE(2)));

                       Float d_hat =
                           PE_d_hat(d_hats(PE(0)), d_hats(PE(1)), d_hats(PE(2)));

                       Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_edge_distance2(flag, P, E0, E1, D);

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

                       Vector9 G;
                       if(gradient_only)
                       {
                           PE_barrier_gradient(G, flag, kt2, d_hat, thickness, P, E0, E1);
                           DoubletVectorAssembler DVA{Gs};
                           DVA.segment<3>(i * 3).write(PE, G);
                       }
                       else
                       {
                           Matrix9x9 H;
                           PE_barrier_gradient_hessian(
                               G, H, flag, kt2, d_hat, thickness, P, E0, E1);
                           make_spd(H);
                           DoubletVectorAssembler DVA{Gs};
                           DVA.segment<3>(i * 3).write(PE, G);
                           TripletMatrixAssembler TMA{Hs};
                           TMA.half_block<3>(i * PEHalfHessianSize).write(PE, H);
                       }
                   });

        // Compute Point-Point Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.PPs().size(),
                   [gradient_only = info.gradient_only(),
                    table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs = info.PPs().viewer().name("PPs"),
                    Gs  = info.PP_gradients().viewer().name("Gs"),
                    Hs  = info.PP_hessians().viewer().name("Hs"),
                    Ps  = info.positions().viewer().name("Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    dt     = info.dt()] __device__(int i) mutable
                   {
                       const auto& PP = PPs(i);

                       Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
                       Float    kt2  = PP_kappa(table, cids) * dt * dt;

                       const auto& P0 = Ps(PP[0]);
                       const auto& P1 = Ps(PP[1]);

                       Float thickness =
                           PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));

                       Float d_hat = PP_d_hat(d_hats(PP(0)), d_hats(PP(1)));

                       Vector2i flag = distance::point_point_distance_flag(P0, P1);

                       if constexpr(RUNTIME_CHECK)
                       {
                           Float D;
                           distance::point_point_distance2(flag, P0, P1, D);

                           Vector2 range = D_range(thickness, d_hat);

                           MUDA_ASSERT(is_active_D(range, D),
                                       "PP[%d,%d] d^2(%f) out of range, (%f,%f)",
                                       PP(0),
                                       PP(1),
                                       D,
                                       range(0),
                                       range(1));
                       }

                       Vector6 G;
                       if(gradient_only)
                       {
                           PP_barrier_gradient(G, flag, kt2, d_hat, thickness, P0, P1);
                           DoubletVectorAssembler DVA{Gs};
                           DVA.segment<2>(i * 2).write(PP, G);
                       }
                       else
                       {
                           Matrix6x6 H;
                           PP_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P0, P1);
                           make_spd(H);
                           DoubletVectorAssembler DVA{Gs};
                           DVA.segment<2>(i * 2).write(PP, G);
                           TripletMatrixAssembler TMA{Hs};
                           TMA.half_block<2>(i * PPHalfHessianSize).write(PP, H);
                       }
                   });
    }
};

REGISTER_SIM_SYSTEM(IPCSimplexNormalContact);
}  // namespace uipc::backend::cuda