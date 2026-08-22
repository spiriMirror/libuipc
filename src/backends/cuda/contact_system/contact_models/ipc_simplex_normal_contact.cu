#include <contact_system/simplex_normal_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/distance/distance_flagged.h>
#include <utils/codim_thickness.h>
#include <kernel_cout.h>
#include <utils/matrix_assembler.h>
#include <utils/make_spd.h>
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
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_simplex_contact;

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

        Float d_hat =
            PT_d_hat(d_hats(PT(0)), d_hats(PT(1)), d_hats(PT(2)), d_hats(PT(3)));

        Vector4i flag = distance::point_triangle_distance_flag(P, T0, T1, T2);

        if constexpr(RUNTIME_CHECK)
        {
            Float D;
            distance::point_triangle_distance2(flag, P, T0, T1, T2, D);

            Vector2 range = D_range(thickness, d_hat);

            UIPC_KERNEL_ASSERT(is_active_D(range, D),
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
    }

    __global__ void do_compute_energy_k2_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector4i> EEs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Vector3> rest_Ps,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_simplex_contact;

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

        Float d_hat =
            EE_d_hat(d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));

        Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

        if constexpr(RUNTIME_CHECK)
        {
            Float D;
            distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);
            Vector2 range = D_range(thickness, d_hat);
            UIPC_KERNEL_ASSERT(is_active_D(range, D),
                               "EE[%d,%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                               EE(0),
                               EE(1),
                               EE(2),
                               EE(3),
                               D,
                               range(0),
                               range(1));
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
    }

    __global__ void do_compute_energy_k3_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector3i> PEs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_simplex_contact;

        Vector3i PE = PEs(i);

        Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
        Float kt2 = PE_kappa(table, cids) * dt * dt;

        const auto& P  = Ps(PE[0]);
        const auto& E0 = Ps(PE[1]);
        const auto& E1 = Ps(PE[2]);

        Float thickness =
            PE_thickness(thicknesses(PE(0)), thicknesses(PE(1)), thicknesses(PE(2)));

        Float d_hat = PE_d_hat(d_hats(PE(0)), d_hats(PE(1)), d_hats(PE(2)));

        Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

        if constexpr(RUNTIME_CHECK)
        {
            Float D;
            distance::point_edge_distance2(flag, P, E0, E1, D);

            Vector2 range = D_range(thickness, d_hat);

            UIPC_KERNEL_ASSERT(is_active_D(range, D),
                               "PE[%d,%d,%d] d^2(%f) out of range, (%f,%f)",
                               PE(0),
                               PE(1),
                               PE(2),
                               D,
                               range(0),
                               range(1));
        }

        Es(i) = PE_barrier_energy(flag, kt2, d_hat, thickness, P, E0, E1);
    }

    __global__ void do_compute_energy_k4_kernel(cuda_tool::CDense2D<ContactCoeff> table,
                                                cuda_tool::CBufferView<IndexT> contact_ids,
                                                cuda_tool::CBufferView<Vector2i> PPs,
                                                cuda_tool::BufferView<Float> Es,
                                                cuda_tool::CBufferView<Vector3> Ps,
                                                cuda_tool::CBufferView<Float> thicknesses,
                                                cuda_tool::CBufferView<Float> d_hats,
                                                Float dt,
                                                int   n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;

        using namespace sym::codim_ipc_simplex_contact;

        Vector2i PP = PPs(i);

        Vector2i cids = {contact_ids(PP[0]), contact_ids(PP[1])};
        Float    kt2  = PP_kappa(table, cids) * dt * dt;

        const auto& Pa = Ps(PP[0]);
        const auto& Pb = Ps(PP[1]);

        Float thickness = PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));

        Float d_hat = PP_d_hat(d_hats(PP(0)), d_hats(PP(1)));

        Vector2i flag = distance::point_point_distance_flag(Pa, Pb);

        if constexpr(RUNTIME_CHECK)
        {
            Float D;
            distance::point_point_distance2(flag, Pa, Pb, D);

            Vector2 range = D_range(thickness, d_hat);

            UIPC_KERNEL_ASSERT(is_active_D(range, D),
                               "PP[%d,%d] d^2(%f) out of range, (%f,%f)",
                               PP(0),
                               PP(1),
                               D,
                               range(0),
                               range(1));
        }

        Es(i) = PP_barrier_energy(flag, kt2, d_hat, thickness, Pa, Pb);
    }

    __global__ void do_assemble_kernel(bool gradient_only,
                                       cuda_tool::CDense2D<ContactCoeff> table,
                                       cuda_tool::CBufferView<IndexT> contact_ids,
                                       cuda_tool::CBufferView<Vector3> Ps,
                                       cuda_tool::CBufferView<Vector3> rest_Ps,
                                       cuda_tool::CBufferView<Float> thicknesses,
                                       cuda_tool::CBufferView<Float>    d_hats,
                                       Float                            dt,
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

        using namespace sym::codim_ipc_simplex_contact;

        if(idx < ee_offset)  // PT
        {
            int      i    = idx;
            Vector4i PT   = PTs(i);
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
            Float d_hat =
                PT_d_hat(d_hats(PT(0)), d_hats(PT(1)), d_hats(PT(2)), d_hats(PT(3)));
            Vector4i flag = distance::point_triangle_distance_flag(P, T0, T1, T2);

            Vector12 G;
            if(gradient_only)
            {
                PT_barrier_gradient(G, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
            }
            else
            {
                Matrix12x12 H;
                PT_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                make_spd(H);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
                TripletMatrixAssembler TMA{PT_Hs};
                TMA.half_block<4>(i * SimplexNormalContact::PTHalfHessianSize).write(PT, H);
            }
        }
        else if(idx < pe_offset)  // EE
        {
            int      i    = idx - ee_offset;
            Vector4i EE   = EEs(i);
            Vector4i cids = {contact_ids(EE[0]),
                             contact_ids(EE[1]),
                             contact_ids(EE[2]),
                             contact_ids(EE[3])};
            Float    kt2  = EE_kappa(table, cids) * dt * dt;

            const auto& E0     = Ps(EE[0]);
            const auto& E1     = Ps(EE[1]);
            const auto& E2     = Ps(EE[2]);
            const auto& E3     = Ps(EE[3]);
            const auto& t0_Ea0 = rest_Ps(EE[0]);
            const auto& t0_Ea1 = rest_Ps(EE[1]);
            const auto& t0_Eb0 = rest_Ps(EE[2]);
            const auto& t0_Eb1 = rest_Ps(EE[3]);

            Float thickness = EE_thickness(thicknesses(EE(0)),
                                           thicknesses(EE(1)),
                                           thicknesses(EE(2)),
                                           thicknesses(EE(3)));
            Float d_hat =
                EE_d_hat(d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));
            Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

            Vector12 G;
            if(gradient_only)
            {
                mollified_EE_barrier_gradient(
                    G, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
            }
            else
            {
                Matrix12x12 H;
                mollified_EE_barrier_gradient_hessian(
                    G, H, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                make_spd(H);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
                TripletMatrixAssembler TMA{EE_Hs};
                TMA.half_block<4>(i * SimplexNormalContact::EEHalfHessianSize).write(EE, H);
            }
        }
        else if(idx < pp_offset)  // PE
        {
            int      i  = idx - pe_offset;
            Vector3i PE = PEs(i);
            Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
            Float kt2 = PE_kappa(table, cids) * dt * dt;

            const auto& P  = Ps(PE[0]);
            const auto& E0 = Ps(PE[1]);
            const auto& E1 = Ps(PE[2]);

            Float thickness =
                PE_thickness(thicknesses(PE(0)), thicknesses(PE(1)), thicknesses(PE(2)));
            Float d_hat = PE_d_hat(d_hats(PE(0)), d_hats(PE(1)), d_hats(PE(2)));
            Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

            Vector9 G;
            if(gradient_only)
            {
                PE_barrier_gradient(G, flag, kt2, d_hat, thickness, P, E0, E1);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
            }
            else
            {
                Matrix9x9 H;
                PE_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P, E0, E1);
                make_spd(H);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
                TripletMatrixAssembler TMA{PE_Hs};
                TMA.half_block<3>(i * SimplexNormalContact::PEHalfHessianSize).write(PE, H);
            }
        }
        else  // PP
        {
            int         i    = idx - pp_offset;
            const auto& PP   = PPs(i);
            Vector2i    cids = {contact_ids(PP[0]), contact_ids(PP[1])};
            Float       kt2  = PP_kappa(table, cids) * dt * dt;

            const auto& P0 = Ps(PP[0]);
            const auto& P1 = Ps(PP[1]);

            Float thickness = PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));
            Float    d_hat = PP_d_hat(d_hats(PP(0)), d_hats(PP(1)));
            Vector2i flag  = distance::point_point_distance_flag(P0, P1);

            Vector6 G;
            if(gradient_only)
            {
                PP_barrier_gradient(G, flag, kt2, d_hat, thickness, P0, P1);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
            }
            else
            {
                Matrix6x6 H;
                PP_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P0, P1);
                make_spd(H);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
                TripletMatrixAssembler TMA{PP_Hs};
                TMA.half_block<2>(i * SimplexNormalContact::PPHalfHessianSize).write(PP, H);
            }
        }
    }
}  // namespace

class IPCSimplexNormalContact final : public SimplexNormalContact
{
  public:
    using SimplexNormalContact::SimplexNormalContact;

    virtual void do_build(BuildInfo& info) override
    {
        require<IPCPipelineFlag>();
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        using namespace cuda_tool;
        using namespace sym::codim_ipc_simplex_contact;

        // Compute Point-Triangle energy
        auto PT_count = info.PTs().size();
        if(PT_count > 0)
            do_compute_energy_k1_kernel<<<cuda_tool::best_grid_dim((int)PT_count, do_compute_energy_k1_kernel), cuda_tool::best_block_dim(do_compute_energy_k1_kernel), 0, nullptr>>>(
                info.contact_tabular().viewer(),
                info.contact_element_ids().viewer(),
                info.PTs().viewer(),
                info.PT_energies().viewer(),
                info.positions().viewer(),
                info.thicknesses().viewer(),
                info.d_hats().viewer(),
                info.dt(),
                (int)PT_count);

        // Compute Edge-Edge energy
        auto EE_count = info.EEs().size();
        if(EE_count > 0)
            do_compute_energy_k2_kernel<<<cuda_tool::best_grid_dim((int)EE_count, do_compute_energy_k2_kernel), cuda_tool::best_block_dim(do_compute_energy_k2_kernel), 0, nullptr>>>(
                info.contact_tabular().viewer(),
                info.contact_element_ids().viewer(),
                info.EEs().viewer(),
                info.EE_energies().viewer(),
                info.positions().viewer(),
                info.thicknesses().viewer(),
                info.rest_positions().viewer(),
                info.d_hats().viewer(),
                info.dt(),
                (int)EE_count);

        // Compute Point-Edge energy
        auto PE_count = info.PEs().size();
        if(PE_count > 0)
            do_compute_energy_k3_kernel<<<cuda_tool::best_grid_dim((int)PE_count, do_compute_energy_k3_kernel), cuda_tool::best_block_dim(do_compute_energy_k3_kernel), 0, nullptr>>>(
                info.contact_tabular().viewer(),
                info.contact_element_ids().viewer(),
                info.PEs().viewer(),
                info.PE_energies().viewer(),
                info.positions().viewer(),
                info.thicknesses().viewer(),
                info.d_hats().viewer(),
                info.dt(),
                (int)PE_count);

        // Compute Point-Point energy
        auto PP_count = info.PPs().size();
        if(PP_count > 0)
            do_compute_energy_k4_kernel<<<cuda_tool::best_grid_dim((int)PP_count, do_compute_energy_k4_kernel), cuda_tool::best_block_dim(do_compute_energy_k4_kernel), 0, nullptr>>>(
                info.contact_tabular().viewer(),
                info.contact_element_ids().viewer(),
                info.PPs().viewer(),
                info.PP_energies().viewer(),
                info.positions().viewer(),
                info.thicknesses().viewer(),
                info.d_hats().viewer(),
                info.dt(),
                (int)PP_count);
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        using namespace cuda_tool;
        using namespace sym::codim_ipc_simplex_contact;

        // Fused kernel: PT + EE + PE + PP in one launch using offset-based dispatch.
        // Reduces 4 kernel launches to 1, improving GPU occupancy by providing
        // more threads in a single launch.
        auto pt_count = (IndexT)info.PTs().size();
        auto ee_count = (IndexT)info.EEs().size();
        auto pe_count = (IndexT)info.PEs().size();
        auto pp_count = (IndexT)info.PPs().size();
        auto total    = pt_count + ee_count + pe_count + pp_count;

        if(total == 0)
            return;

        IndexT ee_offset = pt_count;
        IndexT pe_offset = ee_offset + ee_count;
        IndexT pp_offset = pe_offset + pe_count;

        do_assemble_kernel<<<cuda_tool::best_grid_dim(total, do_assemble_kernel), cuda_tool::best_block_dim(do_assemble_kernel), 0, nullptr>>>(
            info.gradient_only(),
            info.contact_tabular().viewer(),
            info.contact_element_ids().viewer(),
            info.positions().viewer(),
            info.rest_positions().viewer(),
            info.thicknesses().viewer(),
            info.d_hats().viewer(),
            info.dt(),
            // PT
            info.PTs().viewer(),
            info.PT_gradients().viewer(),
            info.PT_hessians().viewer(),
            // EE
            info.EEs().viewer(),
            info.EE_gradients().viewer(),
            info.EE_hessians().viewer(),
            // PE
            info.PEs().viewer(),
            info.PE_gradients().viewer(),
            info.PE_hessians().viewer(),
            // PP
            info.PPs().viewer(),
            info.PP_gradients().viewer(),
            info.PP_hessians().viewer(),
            // offsets
            ee_offset,
            pe_offset,
            pp_offset,
            total);
    }
};

REGISTER_SIM_SYSTEM(IPCSimplexNormalContact);
}  // namespace uipc::backend::cuda