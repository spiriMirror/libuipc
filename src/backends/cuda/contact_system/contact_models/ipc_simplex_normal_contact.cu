/*
 * NVIDIA contributions: contact-type dispatch/grouping, optimized Hessian
 * assembly and fused normal-energy launch integration; assembly and energy
 * launches on the provided streams; the prelaunched trial energy and the
 * one-warp-per-block full-Hessian assembly with its reported block count.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <contact_system/simplex_normal_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <contact_system/contact_models/ipc_simplex_normal_contact_energy.h>
#include <contact_system/contact_models/ipc_simplex_normal_contact_assembly.h>
#include <utils/distance/distance_flagged.h>
#include <utils/codim_thickness.h>
#include <utils/fixed_bank_soa_evd.h>
#include <utils/four_vertex_translation_free_spd.h>
#include <utils/contact_type_block_layout.h>
#include <kernel_cout.h>
#include <utils/matrix_assembler.h>
#include <utils/primitive_d_hat.h>
#include <pipeline/ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    template <bool GradientOnly>
    __global__ void do_assemble_kernel(cuda_tool::CDense2D<ContactCoeff> table,
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
                                       IndexT pt_end,
                                       IndexT ee_offset,
                                       IndexT ee_end,
                                       IndexT pe_offset,
                                       IndexT pe_end,
                                       IndexT pp_offset,
                                       int    n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;

        constexpr int SharedLanePitch = IPCSimplexNormalContactFullHessianBlockSize;
        __shared__ Float shared_h[GradientOnly ? 1 : 12 * 12 * SharedLanePitch];

        using namespace sym::codim_ipc_simplex_contact;

        if(idx < pt_end)  // PT
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
            if constexpr(GradientOnly)
            {
                PT_barrier_gradient(G, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
            }
            else
            {
                FixedBankSoAMap<12, SharedLanePitch> H(shared_h + threadIdx.x);
                Vector12                             eigen_values;
                PT_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P, T0, T1, T2);
                selfadjoint_evd_four_vertex_translation_free_fixed_bank<SharedLanePitch>(
                    H, eigen_values);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
                TripletMatrixAssembler TMA{PT_Hs};
                TMA.half_block<4>(i * SimplexNormalContact::PTHalfHessianSize)
                    .write_psd_from_eigendecomposition(PT, H, eigen_values);
            }
        }
        else if(idx < ee_offset)
        {
            return;
        }
        else if(idx < ee_end)  // EE
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
            if constexpr(GradientOnly)
            {
                mollified_EE_barrier_gradient(
                    G, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
            }
            else
            {
                FixedBankSoAMap<12, SharedLanePitch> H(shared_h + threadIdx.x);
                Vector12                             eigen_values;
                mollified_EE_barrier_gradient_hessian(
                    G, H, flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
                selfadjoint_evd_four_vertex_translation_free_fixed_bank<SharedLanePitch>(
                    H, eigen_values);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
                TripletMatrixAssembler TMA{EE_Hs};
                TMA.half_block<4>(i * SimplexNormalContact::EEHalfHessianSize)
                    .write_psd_from_eigendecomposition(EE, H, eigen_values);
            }
        }
        else if(idx < pe_offset)
        {
            return;
        }
        else if(idx < pe_end)  // PE
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
            if constexpr(GradientOnly)
            {
                PE_barrier_gradient(G, flag, kt2, d_hat, thickness, P, E0, E1);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
            }
            else
            {
                FixedBankSoAMap<9, SharedLanePitch> H(shared_h + threadIdx.x);
                Vector9                             eigen_values;
                PE_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P, E0, E1);
                selfadjoint_evd_fixed_bank_shared<9>(H, eigen_values);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
                TripletMatrixAssembler TMA{PE_Hs};
                TMA.half_block<3>(i * SimplexNormalContact::PEHalfHessianSize)
                    .write_psd_from_eigendecomposition(PE, H, eigen_values);
            }
        }
        else if(idx < pp_offset)
        {
            return;
        }
        else
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
            if constexpr(GradientOnly)
            {
                PP_barrier_gradient(G, flag, kt2, d_hat, thickness, P0, P1);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
            }
            else
            {
                FixedBankSoAMap<6, SharedLanePitch> H(shared_h + threadIdx.x);
                Vector6                             eigen_values;
                PP_barrier_gradient_hessian(G, H, flag, kt2, d_hat, thickness, P0, P1);
                selfadjoint_evd_fixed_bank_shared<6>(H, eigen_values);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
                TripletMatrixAssembler TMA{PP_Hs};
                TMA.half_block<2>(i * SimplexNormalContact::PPHalfHessianSize)
                    .write_psd_from_eigendecomposition(PP, H, eigen_values);
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

    // A trial energy is launched by the capacity of the pending active set
    // before the read-back of its counts.
    virtual bool do_report_prelaunch_energy_extent(GlobalContactManager::PrelaunchEnergyExtentInfo& info) override
    {
        const auto& pending = pending_active_set();
        if(!pending.PT_count)
            return false;
        info.pending_energy_count(pending.PTs.size() + pending.EEs.size()
                                  + pending.PEs.size() + pending.PPs.size());
        return true;
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        IPCSimplexNormalContactEnergyLaunchInfo launch_info{
            .contact_tabular     = info.contact_tabular(),
            .contact_element_ids = info.contact_element_ids(),
            .positions           = info.positions(),
            .rest_positions      = info.rest_positions(),
            .thicknesses         = info.thicknesses(),
            .d_hats              = info.d_hats(),
            .PTs                 = info.PTs(),
            .EEs                 = info.EEs(),
            .PEs                 = info.PEs(),
            .PPs                 = info.PPs(),
            .energies            = info.energies(),
            .dt                  = info.dt(),
            .stream              = info.stream()};
        if(info.prelaunch())
        {
            const auto& pending  = pending_active_set();
            launch_info.PTs      = pending.PTs;
            launch_info.EEs      = pending.EEs;
            launch_info.PEs      = pending.PEs;
            launch_info.PPs      = pending.PPs;
            launch_info.PT_count = pending.PT_count;
            launch_info.EE_count = pending.EE_count;
            launch_info.PE_count = pending.PE_count;
            launch_info.PP_count = pending.PP_count;
        }
        launch_ipc_simplex_normal_contact_energy(launch_info);
    }

    virtual SizeT do_report_assembly_blocks() override
    {
        return ipc_simplex_normal_contact_full_hessian_blocks(
            PTs().size(), EEs().size(), PEs().size(), PPs().size());
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        launch_ipc_simplex_normal_contact_assembly(IPCSimplexNormalContactAssemblyLaunchInfo{
            .contact_tabular     = info.contact_tabular(),
            .contact_element_ids = info.contact_element_ids(),
            .positions           = info.positions(),
            .rest_positions      = info.rest_positions(),
            .thicknesses         = info.thicknesses(),
            .d_hats              = info.d_hats(),
            .PTs                 = info.PTs(),
            .EEs                 = info.EEs(),
            .PEs                 = info.PEs(),
            .PPs                 = info.PPs(),
            .PT_gradients        = info.PT_gradients(),
            .PT_hessians         = info.PT_hessians(),
            .EE_gradients        = info.EE_gradients(),
            .EE_hessians         = info.EE_hessians(),
            .PE_gradients        = info.PE_gradients(),
            .PE_hessians         = info.PE_hessians(),
            .PP_gradients        = info.PP_gradients(),
            .PP_hessians         = info.PP_hessians(),
            .dt                  = info.dt(),
            .gradient_only       = info.gradient_only(),
            .stream              = info.stream()});
    }
};

void launch_ipc_simplex_normal_contact_assembly(const IPCSimplexNormalContactAssemblyLaunchInfo& info)
{
    using namespace cuda_tool;
    using namespace sym::codim_ipc_simplex_contact;

    constexpr SizeT IndexMax = static_cast<SizeT>(std::numeric_limits<IndexT>::max());
    UIPC_ASSERT(info.PTs.size() <= IndexMax && info.EEs.size() <= IndexMax
                    && info.PEs.size() <= IndexMax && info.PPs.size() <= IndexMax,
                "Simplex normal contact count exceeds the IndexT limit: PT={}, EE={}, PE={}, PP={}",
                info.PTs.size(),
                info.EEs.size(),
                info.PEs.size(),
                info.PPs.size());

    auto pt_count = static_cast<IndexT>(info.PTs.size());
    auto ee_count = static_cast<IndexT>(info.EEs.size());
    auto pe_count = static_cast<IndexT>(info.PEs.size());
    auto pp_count = static_cast<IndexT>(info.PPs.size());

    const std::uint64_t total_wide =
        static_cast<std::uint64_t>(pt_count) + static_cast<std::uint64_t>(ee_count)
        + static_cast<std::uint64_t>(pe_count) + static_cast<std::uint64_t>(pp_count);
    UIPC_ASSERT(total_wide <= static_cast<std::uint64_t>(IndexMax),
                "Simplex normal contact total {} exceeds the IndexT limit {}",
                total_wide,
                IndexMax);
    const auto total = static_cast<IndexT>(total_wide);

    constexpr int FullHessianBlockSize = IPCSimplexNormalContactFullHessianBlockSize;
    const auto padded_layout = make_contact_type_block_layout<FullHessianBlockSize>(
        pt_count, ee_count, pe_count, pp_count);

    if(total == 0)
        return;

    // Keep all contact types in one launch: rare PT/EE Hessians are
    // individually expensive, and splitting them serializes work that the
    // fused launch overlaps with the dominant PE population. Specialize
    // only the uniform gradient/Hessian branch.
    auto launch = [&]<bool GradientOnly>()
    {
        auto         k      = do_assemble_kernel<GradientOnly>;
        const IndexT pt_end = pt_count;
        const IndexT ee_offset = GradientOnly ? pt_count : padded_layout.ee_offset;
        const IndexT ee_end = ee_offset + ee_count;
        const IndexT pe_offset = GradientOnly ? ee_end : padded_layout.pe_offset;
        const IndexT pe_end = pe_offset + pe_count;
        const IndexT pp_offset = GradientOnly ? pe_end : padded_layout.pp_offset;
        const int launch_size = GradientOnly ? total : padded_layout.padded_total;
        const int block_size = GradientOnly ? cuda_tool::best_block_dim(k) : FullHessianBlockSize;
        const int grid_size = launch_size / block_size + (launch_size % block_size != 0);
        k<<<grid_size, block_size, 0, info.stream>>>(info.contact_tabular.viewer(),
                                                     info.contact_element_ids.viewer(),
                                                     info.positions.viewer(),
                                                     info.rest_positions.viewer(),
                                                     info.thicknesses.viewer(),
                                                     info.d_hats.viewer(),
                                                     info.dt,
                                                     info.PTs.viewer(),
                                                     info.PT_gradients.viewer(),
                                                     info.PT_hessians.viewer(),
                                                     info.EEs.viewer(),
                                                     info.EE_gradients.viewer(),
                                                     info.EE_hessians.viewer(),
                                                     info.PEs.viewer(),
                                                     info.PE_gradients.viewer(),
                                                     info.PE_hessians.viewer(),
                                                     info.PPs.viewer(),
                                                     info.PP_gradients.viewer(),
                                                     info.PP_hessians.viewer(),
                                                     pt_end,
                                                     ee_offset,
                                                     ee_end,
                                                     pe_offset,
                                                     pe_end,
                                                     pp_offset,
                                                     launch_size);
    };

    if(info.gradient_only)
        launch.operator()<true>();
    else
        launch.operator()<false>();
}

REGISTER_SIM_SYSTEM(IPCSimplexNormalContact);
}  // namespace uipc::backend::cuda
