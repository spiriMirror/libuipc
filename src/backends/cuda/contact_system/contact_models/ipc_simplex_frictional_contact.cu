/*
 * NVIDIA contributions: contact-type dispatch/grouping, optimized Hessian
 * assembly and fused friction-energy launch integration; assembly and energy
 * launches on the provided streams and the per-frame cache of the lagged
 * friction quantities.
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

#include <contact_system/simplex_frictional_contact.h>
#include <contact_system/contact_models/codim_ipc_simplex_frictional_contact_function.h>
#include <contact_system/contact_models/ipc_simplex_frictional_contact_energy.h>
#include <contact_system/contact_models/ipc_simplex_frictional_contact_assembly.h>
#include <utils/codim_thickness.h>
#include <utils/fixed_bank_soa_evd.h>
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
                                       cuda_tool::CBufferView<Vector3> prev_Ps,
                                       cuda_tool::CBufferView<IPCSimplexFrictionLagged> lagged,
                                       Float                            eps_v,
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

        constexpr int    SharedLanePitch = 16;
        __shared__ Float shared_h[GradientOnly ? 1 : 12 * 12 * SharedLanePitch];

        using namespace sym::codim_ipc_contact;

        if(idx < ee_offset)
        {
            // PT friction
            int         i     = idx;
            const auto& PT    = PTs(i);
            Vector4i    cids  = {contact_ids(PT[0]),
                                 contact_ids(PT[1]),
                                 contact_ids(PT[2]),
                                 contact_ids(PT[3])};
            auto        coeff = PT_contact_coeff(table, cids);
            Float       mu    = coeff.mu;

            const auto& prev_P  = prev_Ps(PT[0]);
            const auto& prev_T0 = prev_Ps(PT[1]);
            const auto& prev_T1 = prev_Ps(PT[2]);
            const auto& prev_T2 = prev_Ps(PT[3]);
            const auto& P       = Ps(PT[0]);
            const auto& T0      = Ps(PT[1]);
            const auto& T1      = Ps(PT[2]);
            const auto& T2      = Ps(PT[3]);

            // closest point, tangent basis and normal force of this frame
            const FrictionLagged& L = lagged(idx);

            Vector12 G;
            if constexpr(GradientOnly)
            {
                PT_friction_gradient(
                    G, L, mu, eps_v * dt, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
            }
            else
            {
                FixedBankSoAMap<12, SharedLanePitch> H(shared_h + threadIdx.x);
                const bool hessian_is_psd = PT_friction_gradient_hessian(
                    G, H, L, mu, eps_v * dt, prev_P, prev_T0, prev_T1, prev_T2, P, T0, T1, T2);
                DoubletVectorAssembler DVA{PT_Gs};
                DVA.segment<4>(i * 4).write(PT, G);
                TripletMatrixAssembler TMA{PT_Hs};
                auto H_range = TMA.half_block<4>(i * SimplexFrictionalContact::PTHalfHessianSize);
                if(hessian_is_psd)
                    H_range.write_expression(PT, H);
                else
                {
                    Vector12 eigen_values;
                    selfadjoint_evd_fixed_bank_shared<12>(H, eigen_values);
                    H_range.write_psd_from_eigendecomposition(PT, H, eigen_values);
                }
            }
        }
        else if(idx < pe_offset)
        {
            // EE friction
            int         i     = idx - ee_offset;
            const auto& EE    = EEs(i);
            Vector4i    cids  = {contact_ids(EE[0]),
                                 contact_ids(EE[1]),
                                 contact_ids(EE[2]),
                                 contact_ids(EE[3])};
            auto        coeff = EE_contact_coeff(table, cids);
            Float       mu    = coeff.mu;

            const Vector3& prev_Ea0 = prev_Ps(EE[0]);
            const Vector3& prev_Ea1 = prev_Ps(EE[1]);
            const Vector3& prev_Eb0 = prev_Ps(EE[2]);
            const Vector3& prev_Eb1 = prev_Ps(EE[3]);
            const Vector3& Ea0      = Ps(EE[0]);
            const Vector3& Ea1      = Ps(EE[1]);
            const Vector3& Eb0      = Ps(EE[2]);
            const Vector3& Eb1      = Ps(EE[3]);

            // mollifier, closest points, tangent basis and normal force of this frame
            const FrictionLagged& L         = lagged(idx);
            bool                  mollified = friction_lagged_mollified(L);

            Vector12 G;
            if constexpr(GradientOnly)
            {
                if(mollified)
                    G.setZero();
                else
                    EE_friction_gradient(
                        G, L, mu, eps_v * dt, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
            }
            else
            {
                FixedBankSoAMap<12, SharedLanePitch> H(shared_h + threadIdx.x);
                bool                                 hessian_is_psd = mollified;
                if(mollified)
                {
                    G.setZero();
                    H.setZero();
                }
                else
                {
                    hessian_is_psd = EE_friction_gradient_hessian(
                        G, H, L, mu, eps_v * dt, prev_Ea0, prev_Ea1, prev_Eb0, prev_Eb1, Ea0, Ea1, Eb0, Eb1);
                }
                DoubletVectorAssembler DVA{EE_Gs};
                DVA.segment<4>(i * 4).write(EE, G);
                TripletMatrixAssembler TMA{EE_Hs};
                auto H_range = TMA.half_block<4>(i * SimplexFrictionalContact::EEHalfHessianSize);
                if(hessian_is_psd)
                    H_range.write_expression(EE, H);
                else
                {
                    Vector12 eigen_values;
                    selfadjoint_evd_fixed_bank_shared<12>(H, eigen_values);
                    H_range.write_psd_from_eigendecomposition(EE, H, eigen_values);
                }
            }
        }
        else if(idx < pp_offset)
        {
            // PE friction
            int         i  = idx - pe_offset;
            const auto& PE = PEs(i);
            Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
            auto  coeff = PE_contact_coeff(table, cids);
            Float mu    = coeff.mu;

            const Vector3& prev_P  = prev_Ps(PE[0]);
            const Vector3& prev_E0 = prev_Ps(PE[1]);
            const Vector3& prev_E1 = prev_Ps(PE[2]);
            const Vector3& P       = Ps(PE[0]);
            const Vector3& E0      = Ps(PE[1]);
            const Vector3& E1      = Ps(PE[2]);

            // closest point, tangent basis and normal force of this frame
            const FrictionLagged& L = lagged(idx);

            Vector9 G;
            if constexpr(GradientOnly)
            {
                PE_friction_gradient(G, L, mu, eps_v * dt, prev_P, prev_E0, prev_E1, P, E0, E1);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
            }
            else
            {
                FixedBankSoAMap<9, SharedLanePitch> H(shared_h + threadIdx.x);
                const bool hessian_is_psd = PE_friction_gradient_hessian(
                    G, H, L, mu, eps_v * dt, prev_P, prev_E0, prev_E1, P, E0, E1);
                DoubletVectorAssembler DVA{PE_Gs};
                DVA.segment<3>(i * 3).write(PE, G);
                TripletMatrixAssembler TMA{PE_Hs};
                auto H_range = TMA.half_block<3>(i * SimplexFrictionalContact::PEHalfHessianSize);
                if(hessian_is_psd)
                    H_range.write_expression(PE, H);
                else
                {
                    Vector9 eigen_values;
                    selfadjoint_evd_fixed_bank_shared<9>(H, eigen_values);
                    H_range.write_psd_from_eigendecomposition(PE, H, eigen_values);
                }
            }
        }
        else
        {
            // PP friction
            int         i     = idx - pp_offset;
            const auto& PP    = PPs(i);
            Vector2i    cids  = {contact_ids(PP[0]), contact_ids(PP[1])};
            auto        coeff = PP_contact_coeff(table, cids);
            Float       mu    = coeff.mu;

            const Vector3& prev_P0 = prev_Ps(PP[0]);
            const Vector3& prev_P1 = prev_Ps(PP[1]);
            const Vector3& P0      = Ps(PP[0]);
            const Vector3& P1      = Ps(PP[1]);

            // tangent basis and normal force of this frame
            const FrictionLagged& L = lagged(idx);

            Vector6 G;
            if constexpr(GradientOnly)
            {
                PP_friction_gradient(G, L, mu, eps_v * dt, prev_P0, prev_P1, P0, P1);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
            }
            else
            {
                FixedBankSoAMap<6, SharedLanePitch> H(shared_h + threadIdx.x);
                const bool hessian_is_psd = PP_friction_gradient_hessian(
                    G, H, L, mu, eps_v * dt, prev_P0, prev_P1, P0, P1);
                DoubletVectorAssembler DVA{PP_Gs};
                DVA.segment<2>(i * 2).write(PP, G);
                TripletMatrixAssembler TMA{PP_Hs};
                auto H_range = TMA.half_block<2>(i * SimplexFrictionalContact::PPHalfHessianSize);
                if(hessian_is_psd)
                    H_range.write_expression(PP, H);
                else
                {
                    Vector6 eigen_values;
                    selfadjoint_evd_fixed_bank_shared<6>(H, eigen_values);
                    H_range.write_psd_from_eigendecomposition(PP, H, eigen_values);
                }
            }
        }
    }

}  // namespace

void launch_ipc_simplex_frictional_contact_assembly(const IPCSimplexFrictionalContactAssemblyLaunchInfo& info)
{
    const auto layout = make_contact_type_contiguous_layout<IndexT>(
        info.PTs.size(), info.EEs.size(), info.PEs.size(), info.PPs.size());
    const IndexT total = layout.pp_end;
    if(total == 0)
        return;

    if(info.update_lagged)
        launch_ipc_simplex_frictional_contact_lagged(IPCSimplexFrictionalContactLaggedLaunchInfo{
            .contact_tabular     = info.contact_tabular,
            .contact_element_ids = info.contact_element_ids,
            .prev_positions      = info.prev_positions,
            .rest_positions      = info.rest_positions,
            .thicknesses         = info.thicknesses,
            .d_hats              = info.d_hats,
            .PTs                 = info.PTs,
            .EEs                 = info.EEs,
            .PEs                 = info.PEs,
            .PPs                 = info.PPs,
            .lagged              = info.lagged,
            .dt                  = info.dt,
            .stream              = info.stream});

    // Keep the production contiguous PT | EE | PE | PP dispatch and launch
    // geometry in one place, shared by the system and focused tests.
    auto launch = [&]<bool GradientOnly>()
    {
        auto      k          = do_assemble_kernel<GradientOnly>;
        const int block_size = GradientOnly ? cuda_tool::best_block_dim(k) : 12;
        const int grid_size  = GradientOnly ?
                                   cuda_tool::best_grid_dim(total, k) :
                                   total / block_size + (total % block_size != 0);
        k<<<grid_size, block_size, 0, info.stream>>>(info.contact_tabular.viewer(),
                                                     info.contact_element_ids.viewer(),
                                                     info.positions.viewer(),
                                                     info.prev_positions.viewer(),
                                                     info.lagged,
                                                     info.eps_velocity,
                                                     info.dt,
                                                     info.PTs.viewer(),
                                                     info.PT_gradients,
                                                     info.PT_hessians,
                                                     info.EEs.viewer(),
                                                     info.EE_gradients,
                                                     info.EE_hessians,
                                                     info.PEs.viewer(),
                                                     info.PE_gradients,
                                                     info.PE_hessians,
                                                     info.PPs.viewer(),
                                                     info.PP_gradients,
                                                     info.PP_hessians,
                                                     layout.pt_end,
                                                     layout.ee_end,
                                                     layout.pe_end,
                                                     total);
    };

    if(info.gradient_only)
        launch.operator()<true>();
    else
        launch.operator()<false>();
}

class IPCSimplexFrictionalContact final : public SimplexFrictionalContact
{
  public:
    using SimplexFrictionalContact::SimplexFrictionalContact;

    virtual void do_build(BuildInfo& info) override
    {
        require<IPCPipelineFlag>();
    }

    // Lagged friction quantities of the friction pairs (contiguous PT | EE |
    // PE | PP), recomputed by the first energy or assembly launch after the
    // trajectory filter recorded new friction candidates or the contact
    // tabular changed (the adaptive kappa). Nothing else they depend on
    // (frame-start positions, thickness, d_hat, dt) changes within the frame.
    cuda_tool::DeviceBuffer<IPCSimplexFrictionLagged> lagged;
    SizeT lagged_candidates_version = ~0ull;
    SizeT lagged_tabular_version    = ~0ull;

    bool update_lagged(const BaseInfo& info)
    {
        auto candidates_version = info.friction_candidates_version();
        auto tabular_version    = info.contact_tabular_version();
        if(candidates_version == lagged_candidates_version && tabular_version == lagged_tabular_version)
            return false;
        lagged_candidates_version = candidates_version;
        lagged_tabular_version    = tabular_version;
        lagged.resize_discard(info.friction_PTs().size() + info.friction_EEs().size()
                              + info.friction_PEs().size() + info.friction_PPs().size());
        return true;
    }

    virtual void do_compute_energy(EnergyInfo& info) override
    {
        bool update = update_lagged(info);
        launch_ipc_simplex_frictional_contact_energy(IPCSimplexFrictionalContactEnergyLaunchInfo{
            .contact_tabular     = info.contact_tabular(),
            .contact_element_ids = info.contact_element_ids(),
            .positions           = info.positions(),
            .prev_positions      = info.prev_positions(),
            .rest_positions      = info.rest_positions(),
            .thicknesses         = info.thicknesses(),
            .d_hats              = info.d_hats(),
            .PTs                 = info.friction_PTs(),
            .EEs                 = info.friction_EEs(),
            .PEs                 = info.friction_PEs(),
            .PPs                 = info.friction_PPs(),
            .PT_energies         = info.friction_PT_energies(),
            .EE_energies         = info.friction_EE_energies(),
            .PE_energies         = info.friction_PE_energies(),
            .PP_energies         = info.friction_PP_energies(),
            .lagged              = lagged.view(),
            .update_lagged       = update,
            .eps_velocity        = info.eps_velocity(),
            .dt                  = info.dt(),
            .stream              = info.stream()});
    }

    virtual void do_assemble(ContactInfo& info) override
    {
        bool update = update_lagged(info);
        launch_ipc_simplex_frictional_contact_assembly(IPCSimplexFrictionalContactAssemblyLaunchInfo{
            .contact_tabular     = info.contact_tabular(),
            .contact_element_ids = info.contact_element_ids(),
            .positions           = info.positions(),
            .prev_positions      = info.prev_positions(),
            .rest_positions      = info.rest_positions(),
            .thicknesses         = info.thicknesses(),
            .d_hats              = info.d_hats(),
            .PTs                 = info.friction_PTs(),
            .EEs                 = info.friction_EEs(),
            .PEs                 = info.friction_PEs(),
            .PPs                 = info.friction_PPs(),
            .PT_gradients        = info.friction_PT_gradients(),
            .PT_hessians         = info.friction_PT_hessians(),
            .EE_gradients        = info.friction_EE_gradients(),
            .EE_hessians         = info.friction_EE_hessians(),
            .PE_gradients        = info.friction_PE_gradients(),
            .PE_hessians         = info.friction_PE_hessians(),
            .PP_gradients        = info.friction_PP_gradients(),
            .PP_hessians         = info.friction_PP_hessians(),
            .lagged              = lagged.view(),
            .update_lagged       = update,
            .eps_velocity        = info.eps_velocity(),
            .dt                  = info.dt(),
            .gradient_only       = info.gradient_only(),
            .stream              = info.stream()});
    }
};

REGISTER_SIM_SYSTEM(IPCSimplexFrictionalContact);
}  // namespace uipc::backend::cuda
