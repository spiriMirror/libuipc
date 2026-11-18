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

#pragma once

#include <uipc/backend/macro.h>
#include <contact_system/contact_coeff.h>
#include <cuda_tool/buffer.h>
#include <cuda_tool/linear_system/views.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
struct IPCSimplexNormalContactAssemblyLaunchInfo
{
    cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular;
    cuda_tool::CBufferView<IndexT>         contact_element_ids;
    cuda_tool::CBufferView<Vector3>        positions;
    cuda_tool::CBufferView<Vector3>        rest_positions;
    cuda_tool::CBufferView<Float>          thicknesses;
    cuda_tool::CBufferView<Float>          d_hats;

    cuda_tool::CBufferView<Vector4i> PTs;
    cuda_tool::CBufferView<Vector4i> EEs;
    cuda_tool::CBufferView<Vector3i> PEs;
    cuda_tool::CBufferView<Vector2i> PPs;

    cuda_tool::DoubletVectorView<Float, 3> PT_gradients;
    cuda_tool::TripletMatrixView<Float, 3> PT_hessians;
    cuda_tool::DoubletVectorView<Float, 3> EE_gradients;
    cuda_tool::TripletMatrixView<Float, 3> EE_hessians;
    cuda_tool::DoubletVectorView<Float, 3> PE_gradients;
    cuda_tool::TripletMatrixView<Float, 3> PE_hessians;
    cuda_tool::DoubletVectorView<Float, 3> PP_gradients;
    cuda_tool::TripletMatrixView<Float, 3> PP_hessians;

    Float dt            = 0.0;
    bool  gradient_only = false;
    // every launch goes here (the legacy stream if null)
    cudaStream_t stream = nullptr;
};

// Full-Hessian launch: one-warp blocks, each type's range padded to whole
// blocks, so no block mixes contact types. GB20x issues an FP64 warp
// instruction at the same cost however many lanes are active, so whole warps
// take a quarter of the FP64 issue of eight-lane blocks for the same
// latency-bound kernel, on a quarter of the SMs (the rest stay free for the
// elastic producer, see GlobalLinearSystem::prelaunch_ahead()).
constexpr int IPCSimplexNormalContactFullHessianBlockSize = 32;

// Blocks of the full-Hessian launch for these contact counts.
inline SizeT ipc_simplex_normal_contact_full_hessian_blocks(SizeT pt_count,
                                                            SizeT ee_count,
                                                            SizeT pe_count,
                                                            SizeT pp_count)
{
    constexpr SizeT B = IPCSimplexNormalContactFullHessianBlockSize;
    return (pt_count + B - 1) / B + (ee_count + B - 1) / B
           + (pe_count + B - 1) / B + (pp_count + B - 1) / B;
}

// The production dispatcher: full Hessians use CTA32/type-padded ranges;
// gradient-only uses contiguous ranges and never accesses Hessian views.
UIPC_BACKEND_API void launch_ipc_simplex_normal_contact_assembly(
    const IPCSimplexNormalContactAssemblyLaunchInfo& info);
}  // namespace uipc::backend::cuda
