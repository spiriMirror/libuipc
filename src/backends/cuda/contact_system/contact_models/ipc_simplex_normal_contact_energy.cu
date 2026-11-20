/*
 * NVIDIA contributions: fusion of the existing PT/EE/PE/PP normal
 * contact-energy launches, contact-type dispatch, launch-interface extraction
 * and zero-work handling.
 *
 * The per-contact energy computations are adapted from libuipc's
 * ipc_simplex_normal_contact.cu; those pre-existing portions retain their
 * original provenance.
 *
 * The NVIDIA notice below applies to the modifications described above, not
 * to the pre-existing libuipc code.
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

#include <contact_system/contact_models/ipc_simplex_normal_contact_energy.h>

#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/codim_thickness.h>
#include <utils/distance/distance_flagged.h>
#include <utils/primitive_d_hat.h>
#include <utils/contact_type_block_layout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void ipc_simplex_normal_contact_energy_kernel(
        cuda_tool::CDense2D<ContactCoeff> table,
        cuda_tool::CBufferView<IndexT>    contact_ids,
        cuda_tool::CBufferView<Vector3>   Ps,
        cuda_tool::CBufferView<Vector3>   rest_Ps,
        cuda_tool::CBufferView<Float>     thicknesses,
        cuda_tool::CBufferView<Float>     d_hats,
        cuda_tool::CBufferView<Vector4i>  PTs,
        cuda_tool::CBufferView<Vector4i>  EEs,
        cuda_tool::CBufferView<Vector3i>  PEs,
        cuda_tool::CBufferView<Vector2i>  PPs,
        cuda_tool::BufferView<Float>      PT_Es,
        cuda_tool::BufferView<Float>      EE_Es,
        cuda_tool::BufferView<Float>      PE_Es,
        cuda_tool::BufferView<Float>      PP_Es,
        Float                             dt,
        IndexT                            pt_end,
        IndexT                            ee_end,
        IndexT                            pe_end,
        IndexT                            pp_end)
    {
        IndexT idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= pp_end)
            return;

        using namespace sym::codim_ipc_simplex_contact;

        if(idx < pt_end)
        {
            const IndexT i  = idx;
            Vector4i     PT = PTs(i);

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
                // Only the invariant D > xi^2 (no zero distance / penetration) is
                // checked. Activeness (D < (xi+d_hat)^2) was decided by the collision
                // filter in a separate floating-point evaluation that can differ by an
                // ulp at the upper bound, where the barrier is exactly 0 (KappaBarrier).
                UIPC_KERNEL_ASSERT(D > range(0),
                                   "PT[%d,%d,%d,%d] d^2(%e) <= thickness^2, zero distance or penetration, range (%e,%e)",
                                   PT(0),
                                   PT(1),
                                   PT(2),
                                   PT(3),
                                   D,
                                   range(0),
                                   range(1));
            }

            PT_Es(i) = PT_barrier_energy(flag, kt2, d_hat, thickness, P, T0, T1, T2);
        }
        else if(idx < ee_end)
        {
            const IndexT i  = idx - pt_end;
            Vector4i     EE = EEs(i);

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

            if constexpr(RUNTIME_CHECK)
            {
                Float D;
                distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);
                Vector2 range = D_range(thickness, d_hat);
                // Only the invariant D > xi^2 (no zero distance / penetration) is
                // checked. Activeness (D < (xi+d_hat)^2) was decided by the collision
                // filter in a separate floating-point evaluation that can differ by an
                // ulp at the upper bound, where the barrier is exactly 0 (KappaBarrier).
                UIPC_KERNEL_ASSERT(D > range(0),
                                   "EE[%d,%d,%d,%d] d^2(%e) <= thickness^2, zero distance or penetration, range (%e,%e)",
                                   EE(0),
                                   EE(1),
                                   EE(2),
                                   EE(3),
                                   D,
                                   range(0),
                                   range(1));
            }

            EE_Es(i) = mollified_EE_barrier_energy(
                flag, kt2, d_hat, thickness, t0_Ea0, t0_Ea1, t0_Eb0, t0_Eb1, E0, E1, E2, E3);
        }
        else if(idx < pe_end)
        {
            const IndexT i  = idx - ee_end;
            Vector3i     PE = PEs(i);

            Vector3i cids = {contact_ids(PE[0]), contact_ids(PE[1]), contact_ids(PE[2])};
            Float kt2 = PE_kappa(table, cids) * dt * dt;

            const auto& P  = Ps(PE[0]);
            const auto& E0 = Ps(PE[1]);
            const auto& E1 = Ps(PE[2]);
            Float       thickness =
                PE_thickness(thicknesses(PE(0)), thicknesses(PE(1)), thicknesses(PE(2)));
            Float d_hat = PE_d_hat(d_hats(PE(0)), d_hats(PE(1)), d_hats(PE(2)));
            Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);

            if constexpr(RUNTIME_CHECK)
            {
                Float D;
                distance::point_edge_distance2(flag, P, E0, E1, D);
                Vector2 range = D_range(thickness, d_hat);
                // Only the invariant D > xi^2 (no zero distance / penetration) is
                // checked. Activeness (D < (xi+d_hat)^2) was decided by the collision
                // filter in a separate floating-point evaluation that can differ by an
                // ulp at the upper bound, where the barrier is exactly 0 (KappaBarrier).
                UIPC_KERNEL_ASSERT(D > range(0),
                                   "PE[%d,%d,%d] d^2(%e) <= thickness^2, zero distance or penetration, range (%e,%e)",
                                   PE(0),
                                   PE(1),
                                   PE(2),
                                   D,
                                   range(0),
                                   range(1));
            }

            PE_Es(i) = PE_barrier_energy(flag, kt2, d_hat, thickness, P, E0, E1);
        }
        else
        {
            const IndexT i  = idx - pe_end;
            Vector2i     PP = PPs(i);

            Vector2i    cids = {contact_ids(PP[0]), contact_ids(PP[1])};
            Float       kt2  = PP_kappa(table, cids) * dt * dt;
            const auto& Pa   = Ps(PP[0]);
            const auto& Pb   = Ps(PP[1]);
            Float thickness = PP_thickness(thicknesses(PP(0)), thicknesses(PP(1)));
            Float    d_hat = PP_d_hat(d_hats(PP(0)), d_hats(PP(1)));
            Vector2i flag  = distance::point_point_distance_flag(Pa, Pb);

            if constexpr(RUNTIME_CHECK)
            {
                Float D;
                distance::point_point_distance2(flag, Pa, Pb, D);
                Vector2 range = D_range(thickness, d_hat);
                // Only the invariant D > xi^2 (no zero distance / penetration) is
                // checked. Activeness (D < (xi+d_hat)^2) was decided by the collision
                // filter in a separate floating-point evaluation that can differ by an
                // ulp at the upper bound, where the barrier is exactly 0 (KappaBarrier).
                UIPC_KERNEL_ASSERT(D > range(0),
                                   "PP[%d,%d] d^2(%e) <= thickness^2, zero distance or penetration, range (%e,%e)",
                                   PP(0),
                                   PP(1),
                                   D,
                                   range(0),
                                   range(1));
            }

            PP_Es(i) = PP_barrier_energy(flag, kt2, d_hat, thickness, Pa, Pb);
        }
    }
}  // namespace

void launch_ipc_simplex_normal_contact_energy(const IPCSimplexNormalContactEnergyLaunchInfo& info)
{
    const auto layout = make_contact_type_contiguous_layout<IndexT>(
        info.PTs.size(), info.EEs.size(), info.PEs.size(), info.PPs.size());
    const IndexT pt_end = layout.pt_end;
    const IndexT ee_end = layout.ee_end;
    const IndexT pe_end = layout.pe_end;
    const IndexT pp_end = layout.pp_end;
    if(pp_end == 0)
        return;

    constexpr int BlockSize = 64;
    ipc_simplex_normal_contact_energy_kernel<<<pp_end / BlockSize + (pp_end % BlockSize != 0), BlockSize, 0, nullptr>>>(
        info.contact_tabular.viewer(),
        info.contact_element_ids,
        info.positions,
        info.rest_positions,
        info.thicknesses,
        info.d_hats,
        info.PTs,
        info.EEs,
        info.PEs,
        info.PPs,
        info.PT_energies,
        info.EE_energies,
        info.PE_energies,
        info.PP_energies,
        info.dt,
        pt_end,
        ee_end,
        pe_end,
        pp_end);
}
}  // namespace uipc::backend::cuda
