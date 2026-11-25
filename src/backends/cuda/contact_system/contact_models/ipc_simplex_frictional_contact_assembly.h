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
#include <contact_system/contact_models/ipc_simplex_frictional_contact_energy.h>
#include <cuda_tool/buffer.h>
#include <cuda_tool/linear_system/views.h>

namespace uipc::backend::cuda
{
struct IPCSimplexFrictionalContactAssemblyLaunchInfo
{
    cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular;
    cuda_tool::CBufferView<IndexT>         contact_element_ids;
    cuda_tool::CBufferView<Vector3>        positions;
    cuda_tool::CBufferView<Vector3>        prev_positions;
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

    // The pairs' lagged quantities, as in the energy launch.
    cuda_tool::BufferView<IPCSimplexFrictionLagged> lagged;
    bool                                            update_lagged = true;

    Float eps_velocity  = 0.0;
    Float dt            = 0.0;
    bool  gradient_only = false;
    // every launch goes here (the legacy stream if null)
    cudaStream_t stream = nullptr;
};

// The production dispatch, including its CTA12/Pitch16 full-Hessian kernel.
// Empty total is a no-op. Gradient-only accepts empty Hessian views.
UIPC_BACKEND_API void launch_ipc_simplex_frictional_contact_assembly(
    const IPCSimplexFrictionalContactAssemblyLaunchInfo& info);
}  // namespace uipc::backend::cuda
