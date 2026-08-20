#include <contact_system/simplex_frictional_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <utils/codim_thickness.h>
#include <kernel_cout.h>
#include <utils/make_spd.h>
#include <utils/matrix_assembler.h>
#include <utils/primitive_d_hat.h>
#include <pipeline/ipc_pipeline_flag.h>


namespace uipc::backend::cuda
{
namespace
{
    __global__ void do_compute_energy_k1_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector4i> PTs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Vector3> prev_Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float eps_v,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_contact;

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
        Float d_hat     = PT_d_hat(
            d_hats(PT[0]), d_hats(PT[1]), d_hats(PT[2]), d_hats(PT[3]));


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
    }

    __global__ void do_compute_energy_k2_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector4i> EEs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Vector3> prev_Ps,
                                                cuda_tool::CBufferView<Vector3> rest_Ps,
                                                Float eps_v,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_contact;

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

        Float d_hat = EE_d_hat(
            d_hats(EE[0]), d_hats(EE[1]), d_hats(EE[2]), d_hats(EE[3]));

        Float eps_x;
        distance::edge_edge_mollifier_threshold(
            rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, static_cast<Float>(1e-3), eps_x);
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
    }

    __global__ void do_compute_energy_k3_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector3i> PEs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Vector3> prev_Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float eps_v,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_contact;

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

        Float d_hat =
            PE_d_hat(d_hats(PE[0]), d_hats(PE[1]), d_hats(PE[2]));

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
    }

    __global__ void do_compute_energy_k4_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector2i> PPs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Vector3> prev_Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float eps_v,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_contact;

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

        Float d_hat = PP_d_hat(d_hats(PP[0]), d_hats(PP[1]));

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
    }

    __global__ void do_assemble_kernel(bool gradient_only,
                                       cuda_tool::CDense2D<ContactCoeff> table,
                                       cuda_tool::CBufferView<IndexT> contact_ids,
                                       cuda_tool::CBufferView<Vector3> Ps,
                                       cuda_tool::CBufferView<Vector3> prev_Ps,
                                       cuda_tool::CBufferView<Vector3> rest_Ps,
                                       cuda_tool::CBufferView<Float> thicknesses,
                                       cuda_tool::CBufferView<Float> d_hats,
                                       Float eps_v,
                                       Float dt,
                                       cuda_tool::CBufferView<Vector4i> PTs,
                                       cuda_tool::DoubletVectorView<Float, 3> PT_Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> PT_Hs,
                                       cuda_tool::CBufferView<Vector4i> EEs,
                                       cuda_tool::DoubletVectorView<Float, 3> EE_Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> EE_Hs,
                                       cuda_tool::CBufferView<Vector3i> PEs,
                                       cuda_tool::DoubletVectorView<Float, 3> PE_Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> PE_Hs,
                                       cuda_tool::CBufferView<Vector2i> PPs,
                                       cuda_tool::DoubletVectorView<Float, 3> PP_Gs,
                                       cuda_tool::TripletMatrixView<Float, 3> PP_Hs,
                                       IndexT ee_offset,
                                       IndexT pe_offset,
                                       IndexT pp_offset,
                                       int    n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        using namespace sym::codim_ipc_contact;

        if(idx < ee_offset)
        {
            // PT friction
            int i = idx;
            const auto& PT = PTs(i);
            Vector4i cids = {contact_ids(PT[0]), contact_ids(PT[1]),
                             contact_ids(PT[2]), contact_ids(PT[3])};
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

            Float thickness = PT_thickness(thicknesses(PT[0]), thicknesses(PT[1]),
                                           thicknesses(PT[2]), thicknesses(PT[3]));
            Float d_hat = PT_d_hat(d_hats(PT[0]), d_hats(PT[1]),
                                   d_hats(PT[2]), d_hats(PT[3]));

            Vector12 G;
            if(gradient_only)
            {
                PT_friction_gradient(G, kt2, d_hat, thickness, mu, eps_v * dt,
                                     prev_P, prev_T0, prev_T1, prev_T2,
                                     P, T0, T1, T2);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
            }
            else
            {
                Matrix12x12 H;
                PT_friction_gradient_hessian(G, H, kt2, d_hat, thickness, mu, eps_v * dt,
                                             prev_P, prev_T0, prev_T1, prev_T2,
                                             P, T0, T1, T2);
                cuda::make_spd(H);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
                TripletMatrixAssembler TMA{PT_Hs};
                TMA.half_block<4>(i * SimplexFrictionalContact::PTHalfHessianSize).write(PT, H);
            }
        }
        else if(idx < pe_offset)
        {
            // EE friction
            int i = idx - ee_offset;
            const auto& EE = EEs(i);
            Vector4i cids = {contact_ids(EE[0]), contact_ids(EE[1]),
                             contact_ids(EE[2]), contact_ids(EE[3])};
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

            Float thickness = EE_thickness(thicknesses(EE[0]), thicknesses(EE[1]),
                                           thicknesses(EE[2]), thicknesses(EE[3]));
            Float d_hat = EE_d_hat(d_hats(EE[0]), d_hats(EE[1]),
                                   d_hats(EE[2]), d_hats(EE[3]));

            Float eps_x;
            distance::edge_edge_mollifier_threshold(
                rest_Ea0, rest_Ea1, rest_Eb0, rest_Eb1, static_cast<Float>(1e-3), eps_x);
            bool mollified = distance::need_mollify(prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, eps_x);

            Vector12 G;
            if(gradient_only)
            {
                if(mollified)
                    G.setZero();
                else
                    EE_friction_gradient(G, kt2, d_hat, thickness, mu, eps_v * dt,
                                         prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1,
                                         Ea0, Ea1, Eb0, Eb1);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
            }
            else
            {
                Matrix12x12 H;
                if(mollified)
                {
                    G.setZero();
                    H.setZero();
                }
                else
                {
                    EE_friction_gradient_hessian(G, H, kt2, d_hat, thickness, mu, eps_v * dt,
                                                 prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1,
                                                 Ea0, Ea1, Eb0, Eb1);
                    cuda::make_spd(H);
                }
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
                TripletMatrixAssembler TMA{EE_Hs};
                TMA.half_block<4>(i * SimplexFrictionalContact::EEHalfHessianSize).write(EE, H);
            }
        }
        else if(idx < pp_offset)
        {
            // PE friction
            int i = idx - pe_offset;
            const auto& PE = PEs(i);
            Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
            auto  coeff = PE_contact_coeff(table, cids);
            Float kt2   = coeff.kappa * dt * dt;
            Float mu    = coeff.mu;

            const Vector3& prev_P  = prev_Ps(PE[0]);
            const Vector3& prev_E0 = prev_Ps(PE[1]);
            const Vector3& prev_E1 = prev_Ps(PE[2]);
            const Vector3& P  = Ps(PE[0]);
            const Vector3& E0 = Ps(PE[1]);
            const Vector3& E1 = Ps(PE[2]);

            Float thickness = PE_thickness(thicknesses(PE[0]), thicknesses(PE[1]), thicknesses(PE[2]));
            Float d_hat = PE_d_hat(d_hats(PE[0]), d_hats(PE[1]), d_hats(PE[2]));

            Vector9 G;
            if(gradient_only)
            {
                PE_friction_gradient(G, kt2, d_hat, thickness, mu, eps_v * dt,
                                     prev_P, prev_E0, prev_E1, P, E0, E1);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
            }
            else
            {
                Matrix9x9 H;
                PE_friction_gradient_hessian(G, H, kt2, d_hat, thickness, mu, eps_v * dt,
                                             prev_P, prev_E0, prev_E1, P, E0, E1);
                cuda::make_spd(H);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
                TripletMatrixAssembler TMA{PE_Hs};
                TMA.half_block<3>(i * SimplexFrictionalContact::PEHalfHessianSize).write(PE, H);
            }
        }
        else
        {
            // PP friction
            int i = idx - pp_offset;
            const auto& PP = PPs(i);
            Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
            auto  coeff = PP_contact_coeff(table, cids);
            Float kt2   = coeff.kappa * dt * dt;
            Float mu    = coeff.mu;

            const Vector3& prev_P0 = prev_Ps(PP[0]);
            const Vector3& prev_P1 = prev_Ps(PP[1]);
            const Vector3& P0 = Ps(PP[0]);
            const Vector3& P1 = Ps(PP[1]);

            Float thickness = PP_thickness(thicknesses(PP[0]), thicknesses(PP[1]));
            Float d_hat = PP_d_hat(d_hats(PP[0]), d_hats(PP[1]));

            Vector6 G;
            if(gradient_only)
            {
                PP_friction_gradient(G, kt2, d_hat, thickness, mu, eps_v * dt,
                                     prev_P0, prev_P1, P0, P1);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
            }
            else
            {
                Matrix6x6 H;
                PP_friction_gradient_hessian(G, H, kt2, d_hat, thickness, mu, eps_v * dt,
                                             prev_P0, prev_P1, P0, P1);
                cuda::make_spd(H);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
                TripletMatrixAssembler TMA{PP_Hs};
                TMA.half_block<2>(i * SimplexFrictionalContact::PPHalfHessianSize).write(PP, H);
            }
        }
    }
}  // namespace

class IPCSimplexFrictionalContact final : public SimplexFrictionalContact
{
  public:
    using SimplexFrictionalContact::SimplexFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        require<IPCPipelineFlag>();
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        using namespace cuda_tool;
        using namespace sym::codim_ipc_contact;

        // Compute Point-Triangle energy
        auto PT_count = info.friction_PTs().size();
        if(PT_count > 0)
            do_compute_energy_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_count,
                                                                   do_compute_energy_k1_kernel),
                                          cuda_tool::best_block_dim(do_compute_energy_k1_kernel),
                                          0,
                                          nullptr>>>(info.contact_tabular().viewer(),
                                                     info.contact_element_ids().viewer(),
                                                     info.friction_PTs().viewer(),
                                                     info.friction_PT_energies().viewer(),
                                                     info.positions().viewer(),
                                                     info.prev_positions().viewer(),
                                                     info.thicknesses().viewer(),
                                                     info.d_hats().viewer(),
                                                     info.eps_velocity(),
                                                     info.dt(),
                                                     (int)PT_count);

        // Compute Edge-Edge energy
        auto EE_count = info.friction_EEs().size();
        if(EE_count > 0)
            do_compute_energy_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_count,
                                                                   do_compute_energy_k2_kernel),
                                          cuda_tool::best_block_dim(do_compute_energy_k2_kernel),
                                          0,
                                          nullptr>>>(info.contact_tabular().viewer(),
                                                     info.contact_element_ids().viewer(),
                                                     info.friction_EEs().viewer(),
                                                     info.friction_EE_energies().viewer(),
                                                     info.positions().viewer(),
                                                     info.prev_positions().viewer(),
                                                     info.rest_positions().viewer(),
                                                     info.eps_velocity(),
                                                     info.thicknesses().viewer(),
                                                     info.d_hats().viewer(),
                                                     info.dt(),
                                                     (int)EE_count);

        // Compute Point-Edge energy
        auto PE_count = info.friction_PEs().size();
        if(PE_count > 0)
            do_compute_energy_k3_kernel<<<cuda_tool::best_grid_dim((int)PE_count,
                                                                   do_compute_energy_k3_kernel),
                                          cuda_tool::best_block_dim(do_compute_energy_k3_kernel),
                                          0,
                                          nullptr>>>(info.contact_tabular().viewer(),
                                                     info.contact_element_ids().viewer(),
                                                     info.friction_PEs().viewer(),
                                                     info.friction_PE_energies().viewer(),
                                                     info.positions().viewer(),
                                                     info.prev_positions().viewer(),
                                                     info.thicknesses().viewer(),
                                                     info.d_hats().viewer(),
                                                     info.eps_velocity(),
                                                     info.dt(),
                                                     (int)PE_count);

        // Compute Point-Point energy
        auto PP_count = info.friction_PPs().size();
        if(PP_count > 0)
            do_compute_energy_k4_kernel<<<cuda_tool::best_grid_dim((int)PP_count,
                                                                   do_compute_energy_k4_kernel),
                                          cuda_tool::best_block_dim(do_compute_energy_k4_kernel),
                                          0,
                                          nullptr>>>(info.contact_tabular().viewer(),
                                                     info.contact_element_ids().viewer(),
                                                     info.friction_PPs().viewer(),
                                                     info.friction_PP_energies().viewer(),
                                                     info.positions().viewer(),
                                                     info.prev_positions().viewer(),
                                                     info.thicknesses().viewer(),
                                                     info.d_hats().viewer(),
                                                     info.eps_velocity(),
                                                     info.dt(),
                                                     (int)PP_count);
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace cuda_tool;
        using namespace sym::codim_ipc_contact;

        // Fused kernel: PT + EE + PE + PP friction in one launch
        auto pt_count = (IndexT)info.friction_PTs().size();
        auto ee_count = (IndexT)info.friction_EEs().size();
        auto pe_count = (IndexT)info.friction_PEs().size();
        auto pp_count = (IndexT)info.friction_PPs().size();
        auto total    = pt_count + ee_count + pe_count + pp_count;

        if(total == 0)
            return;

        IndexT ee_offset = pt_count;
        IndexT pe_offset = ee_offset + ee_count;
        IndexT pp_offset = pe_offset + pe_count;

        do_assemble_kernel<<<cuda_tool::best_grid_dim(total, do_assemble_kernel),
                             cuda_tool::best_block_dim(do_assemble_kernel),
                             0,
                             nullptr>>>(info.gradient_only(),
                                        info.contact_tabular().viewer(),
                                        info.contact_element_ids().viewer(),
                                        info.positions().viewer(),
                                        info.prev_positions().viewer(),
                                        info.rest_positions().viewer(),
                                        info.thicknesses().viewer(),
                                        info.d_hats().viewer(),
                                        info.eps_velocity(),
                                        info.dt(),
                                        // PT
                                        info.friction_PTs().viewer(),
                                        info.friction_PT_gradients().viewer(),
                                        info.friction_PT_hessians().viewer(),
                                        // EE
                                        info.friction_EEs().viewer(),
                                        info.friction_EE_gradients().viewer(),
                                        info.friction_EE_hessians().viewer(),
                                        // PE
                                        info.friction_PEs().viewer(),
                                        info.friction_PE_gradients().viewer(),
                                        info.friction_PE_hessians().viewer(),
                                        // PP
                                        info.friction_PPs().viewer(),
                                        info.friction_PP_gradients().viewer(),
                                        info.friction_PP_hessians().viewer(),
                                        // offsets
                                        ee_offset,
                                        pe_offset,
                                        pp_offset,
                                        total);
    }
};

REGISTER_SIM_SYSTEM(IPCSimplexFrictionalContact);
}  // namespace uipc::backend::cuda