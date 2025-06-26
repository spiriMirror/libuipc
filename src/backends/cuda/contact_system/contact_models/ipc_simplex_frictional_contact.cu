#include <contact_system/simplex_frictional_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/codim_thickness.h>
#include <kernel_cout.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>


namespace uipc::backend::cuda
{
class IPCSimplexFrictionalContact final : public SimplexFrictionalContact
{
  public:
    using SimplexFrictionalContact::SimplexFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        auto constitution =
            world().scene().info()["contact"]["constitution"].get<std::string>();
        if(constitution != "ipc")
        {
            throw SimSystemException("Constitution is not IPC");
        }
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        using namespace muda;
        using namespace sym::codim_ipc_contact;

        // Compute Point-Triangle energy
        auto PT_count = info.friction_PTs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(PT_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PTs     = info.friction_PTs().viewer().name("PTs"),
                    Es      = info.friction_PT_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PT = PTs(i);

                       Vector4i cids = {contact_ids(PT[0]),
                                        contact_ids(PT[1]),
                                        contact_ids(PT[2]),
                                        contact_ids(PT[3])};

                       auto  coeff = PT_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const auto& prev_P  = prev_Ps(PT[0]);
                       const auto& prev_T0 = prev_Ps(PT[1]);
                       const auto& prev_T1 = prev_Ps(PT[2]);
                       const auto& prev_T2 = prev_Ps(PT[3]);

                       const auto& P  = Ps(PT[0]);
                       const auto& T0 = Ps(PT[1]);
                       const auto& T1 = Ps(PT[2]);
                       const auto& T2 = Ps(PT[3]);


                       Float thickness = PT_thickness(thicknesses(PT[0]),
                                                      thicknesses(PT[1]),
                                                      thicknesses(PT[2]),
                                                      thicknesses(PT[3]));


                       Es(i) = PT_friction_energy(kt2,
                                                  d_hat,
                                                  thickness,
                                                  mu,
                                                  eps_v * dt,
                                                  // previous positions
                                                  prev_P,
                                                  prev_T0,
                                                  prev_T1,
                                                  prev_T2,
                                                  // current positions
                                                  P,
                                                  T0,
                                                  T1,
                                                  T2);
                   });

        // Compute Edge-Edge energy
        auto EE_count = info.friction_EEs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(EE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    EEs     = info.friction_EEs().viewer().name("EEs"),
                    Es      = info.friction_EE_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    eps_v   = info.eps_velocity(),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& EE = EEs(i);

                       Vector4i cids = {contact_ids(EE[0]),
                                        contact_ids(EE[1]),
                                        contact_ids(EE[2]),
                                        contact_ids(EE[3])};

                       auto  coeff = EE_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const Vector3& rest_Ea0 = rest_Ps(EE[0]);
                       const Vector3& rest_Ea1 = rest_Ps(EE[1]);
                       const Vector3& rest_Eb0 = rest_Ps(EE[2]);
                       const Vector3& rest_Eb1 = rest_Ps(EE[3]);

                       const Vector3& prev_Ea0 = prev_Ps(EE[0]);
                       const Vector3& prev_Ea1 = prev_Ps(EE[1]);
                       const Vector3& prev_Eb0 = prev_Ps(EE[2]);
                       const Vector3& prev_Eb1 = prev_Ps(EE[3]);

                       const Vector3& Ea0 = Ps(EE[0]);
                       const Vector3& Ea1 = Ps(EE[1]);
                       const Vector3& Eb0 = Ps(EE[2]);
                       const Vector3& Eb1 = Ps(EE[3]);

                       Float thickness = EE_thickness(thicknesses(EE[0]),
                                                      thicknesses(EE[1]),
                                                      thicknesses(EE[2]),
                                                      thicknesses(EE[3]));

                       Float eps_x;
                       distance::edge_edge_mollifier_threshold(
                           rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, eps_x);
                       if(distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x))
                       // almost parallel, don't compute energy
                       {
                           Es(i) = 0;
                       }
                       else
                       {
                           Es(i) = EE_friction_energy(kt2,
                                                      d_hat,
                                                      thickness,
                                                      mu,
                                                      eps_v * dt,
                                                      // previous positions
                                                      prev_Ea0,
                                                      prev_Ea1,
                                                      prev_Eb0,
                                                      prev_Eb1,
                                                      // current positions
                                                      Ea0,
                                                      Ea1,
                                                      Eb0,
                                                      Eb1);
                       }
                   });

        // Compute Point-Edge energy
        auto PE_count = info.friction_PEs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(PE_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.friction_PEs().viewer().name("PEs"),
                    Es      = info.friction_PE_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PE = PEs(i);

                       Vector3i cids = {contact_ids(PE[0]),
                                        contact_ids(PE[1]),
                                        contact_ids(PE[2])};

                       auto  coeff = PE_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const Vector3& prev_P  = prev_Ps(PE[0]);
                       const Vector3& prev_E0 = prev_Ps(PE[1]);
                       const Vector3& prev_E1 = prev_Ps(PE[2]);

                       const Vector3& P  = Ps(PE[0]);
                       const Vector3& E0 = Ps(PE[1]);
                       const Vector3& E1 = Ps(PE[2]);

                       Float thickness = PE_thickness(thicknesses(PE[0]),
                                                      thicknesses(PE[1]),
                                                      thicknesses(PE[2]));

                       Es(i) = PE_friction_energy(kt2,
                                                  d_hat,
                                                  thickness,
                                                  mu,
                                                  eps_v * dt,
                                                  // previous positions
                                                  prev_P,
                                                  prev_E0,
                                                  prev_E1,
                                                  // current positions
                                                  P,
                                                  E0,
                                                  E1);
                   });

        // Compute Point-Point energy
        auto PP_count = info.friction_PPs().size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(PP_count,
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs     = info.friction_PPs().viewer().name("PPs"),
                    Es      = info.friction_PP_energies().viewer().name("Es"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PP = PPs(i);

                       Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
                       auto     coeff = PP_contact_coeff(table, cids);
                       Float    kt2   = coeff.kappa * dt * dt;
                       Float    mu    = coeff.mu;

                       const Vector3& prev_P0 = prev_Ps(PP[0]);
                       const Vector3& prev_P1 = prev_Ps(PP[1]);

                       const Vector3& P0 = Ps(PP[0]);
                       const Vector3& P1 = Ps(PP[1]);

                       Float thickness =
                           PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));

                       Es(i) = PP_friction_energy(kt2,
                                                  d_hat,
                                                  thickness,
                                                  mu,
                                                  eps_v * dt,
                                                  // previous positions
                                                  prev_P0,
                                                  prev_P1,
                                                  // current positions
                                                  P0,
                                                  P1);
                   });
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace muda;
        using namespace sym::codim_ipc_contact;

        // Compute Point-Triangle Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.friction_PTs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PTs     = info.friction_PTs().viewer().name("PTs"),
                    Gs      = info.friction_PT_gradients().viewer().name("Gs"),
                    Hs      = info.friction_PT_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PT = PTs(i);

                       Vector4i cids = {contact_ids(PT[0]),
                                        contact_ids(PT[1]),
                                        contact_ids(PT[2]),
                                        contact_ids(PT[3])};

                       auto  coeff = PT_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const auto& prev_P  = prev_Ps(PT[0]);
                       const auto& prev_T0 = prev_Ps(PT[1]);
                       const auto& prev_T1 = prev_Ps(PT[2]);
                       const auto& prev_T2 = prev_Ps(PT[3]);

                       const auto& P  = Ps(PT[0]);
                       const auto& T0 = Ps(PT[1]);
                       const auto& T1 = Ps(PT[2]);
                       const auto& T2 = Ps(PT[3]);


                       Float thickness = PT_thickness(thicknesses(PT[0]),
                                                      thicknesses(PT[1]),
                                                      thicknesses(PT[2]),
                                                      thicknesses(PT[3]));

                       Vector12    G;
                       Matrix12x12 H;

                       PT_friction_gradient_hessian(G,
                                                    H,
                                                    kt2,
                                                    d_hat,
                                                    thickness,
                                                    mu,
                                                    eps_v * dt,
                                                    // previous positions
                                                    prev_P,
                                                    prev_T0,
                                                    prev_T1,
                                                    prev_T2,
                                                    // current positions
                                                    P,
                                                    T0,
                                                    T1,
                                                    T2);

                       cuda::make_spd(H);

                       DoubletVectorAssembler DVA{Gs};
                       DVA.segment<4>(i * 4).write(PT, G);

                       TripletMatrixAssembler TMA{Hs};
                       TMA.block<4, 4>(i * 4 * 4).write(PT, H);
                   });

        // Compute Edge-Edge Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.friction_EEs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    EEs     = info.friction_EEs().viewer().name("EEs"),
                    Gs      = info.friction_EE_gradients().viewer().name("Gs"),
                    Hs      = info.friction_EE_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    rest_Ps = info.rest_positions().viewer().name("rest_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& EE = EEs(i);

                       Vector4i cids = {contact_ids(EE[0]),
                                        contact_ids(EE[1]),
                                        contact_ids(EE[2]),
                                        contact_ids(EE[3])};

                       auto  coeff = EE_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const Vector3& rest_Ea0 = rest_Ps(EE[0]);
                       const Vector3& rest_Ea1 = rest_Ps(EE[1]);
                       const Vector3& rest_Eb0 = rest_Ps(EE[2]);
                       const Vector3& rest_Eb1 = rest_Ps(EE[3]);

                       const Vector3& prev_Ea0 = prev_Ps(EE[0]);
                       const Vector3& prev_Ea1 = prev_Ps(EE[1]);
                       const Vector3& prev_Eb0 = prev_Ps(EE[2]);
                       const Vector3& prev_Eb1 = prev_Ps(EE[3]);

                       const Vector3& Ea0 = Ps(EE[0]);
                       const Vector3& Ea1 = Ps(EE[1]);
                       const Vector3& Eb0 = Ps(EE[2]);
                       const Vector3& Eb1 = Ps(EE[3]);

                       Float thickness = EE_thickness(thicknesses(EE[0]),
                                                      thicknesses(EE[1]),
                                                      thicknesses(EE[2]),
                                                      thicknesses(EE[3]));

                       Float eps_x;
                       distance::edge_edge_mollifier_threshold(
                           rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, eps_x);

                       Vector12    G;
                       Matrix12x12 H;

                       if(distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x))
                       // almost parallel, don't compute gradient and hessian
                       {
                           G.setZero();
                           H.setZero();
                       }
                       else
                       {
                           EE_friction_gradient_hessian(G,
                                                        H,
                                                        kt2,
                                                        d_hat,
                                                        thickness,
                                                        mu,
                                                        eps_v * dt,
                                                        // previous positions
                                                        prev_Ea0,
                                                        prev_Ea1,
                                                        prev_Eb0,
                                                        prev_Eb1,
                                                        // current positions
                                                        Ea0,
                                                        Ea1,
                                                        Eb0,
                                                        Eb1);

                           cuda::make_spd(H);
                       }

                       DoubletVectorAssembler DVA{Gs};
                       DVA.segment<4>(i * 4).write(EE, G);

                       TripletMatrixAssembler TMA{Hs};
                       TMA.block<4, 4>(i * 4 * 4).write(EE, H);
                   });

        // Compute Point-Edge Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.friction_PEs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PEs     = info.friction_PEs().viewer().name("PEs"),
                    Gs      = info.friction_PE_gradients().viewer().name("Gs"),
                    Hs      = info.friction_PE_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PE = PEs(i);

                       Vector3i cids = {contact_ids(PE[0]),
                                        contact_ids(PE[1]),
                                        contact_ids(PE[2])};

                       auto  coeff = PE_contact_coeff(table, cids);
                       Float kt2   = coeff.kappa * dt * dt;
                       Float mu    = coeff.mu;

                       const Vector3& prev_P  = prev_Ps(PE[0]);
                       const Vector3& prev_E0 = prev_Ps(PE[1]);
                       const Vector3& prev_E1 = prev_Ps(PE[2]);

                       const Vector3& P  = Ps(PE[0]);
                       const Vector3& E0 = Ps(PE[1]);
                       const Vector3& E1 = Ps(PE[2]);

                       Float thickness = PE_thickness(thicknesses(PE[0]),
                                                      thicknesses(PE[1]),
                                                      thicknesses(PE[2]));

                       Vector9   G;
                       Matrix9x9 H;

                       PE_friction_gradient_hessian(G,
                                                    H,
                                                    kt2,
                                                    d_hat,
                                                    thickness,
                                                    mu,
                                                    eps_v * dt,
                                                    // previous positions
                                                    prev_P,
                                                    prev_E0,
                                                    prev_E1,
                                                    // current positions
                                                    P,
                                                    E0,
                                                    E1);

                       cuda::make_spd(H);

                       DoubletVectorAssembler DVA{Gs};
                       DVA.segment<3>(i * 3).write(PE, G);

                       TripletMatrixAssembler TMA{Hs};
                       TMA.block<3, 3>(i * 3 * 3).write(PE, H);
                   });

        // Compute Point-Point Gradient and Hessian
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.friction_PPs().size(),
                   [table = info.contact_tabular().viewer().name("contact_tabular"),
                    contact_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                    PPs     = info.friction_PPs().viewer().name("PPs"),
                    Gs      = info.friction_PP_gradients().viewer().name("Gs"),
                    Hs      = info.friction_PP_hessians().viewer().name("Hs"),
                    Ps      = info.positions().viewer().name("Ps"),
                    prev_Ps = info.prev_positions().viewer().name("prev_Ps"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    eps_v = info.eps_velocity(),
                    d_hat = info.d_hat(),
                    dt    = info.dt()] __device__(int i) mutable
                   {
                       const auto& PP = PPs(i);

                       Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
                       auto     coeff = PP_contact_coeff(table, cids);
                       Float    kt2   = coeff.kappa * dt * dt;
                       Float    mu    = coeff.mu;

                       const Vector3& prev_P0 = prev_Ps(PP[0]);
                       const Vector3& prev_P1 = prev_Ps(PP[1]);

                       const Vector3& P0 = Ps(PP[0]);
                       const Vector3& P1 = Ps(PP[1]);

                       Float thickness =
                           PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));


                       Vector6   G;
                       Matrix6x6 H;

                       PP_friction_gradient_hessian(G,
                                                    H,
                                                    kt2,
                                                    d_hat,
                                                    thickness,
                                                    mu,
                                                    eps_v * dt,
                                                    // previous positions
                                                    prev_P0,
                                                    prev_P1,
                                                    // current positions
                                                    P0,
                                                    P1);

                       cuda::make_spd(H);

                       DoubletVectorAssembler DVA{Gs};
                       DVA.segment<2>(i * 2).write(PP, G);

                       TripletMatrixAssembler TMA{Hs};
                       TMA.block<2, 2>(i * 2 * 2).write(PP, H);
                   });
    }
};


REGISTER_SIM_SYSTEM(IPCSimplexFrictionalContact);
}  // namespace uipc::backend::cuda