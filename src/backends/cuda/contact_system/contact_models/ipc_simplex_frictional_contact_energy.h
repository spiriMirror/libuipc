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

#include <contact_system/contact_coeff.h>
#include <cuda_tool/buffer.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
// Lagged friction quantities of one pair, contiguous over PT | EE | PE | PP
// (sym::codim_ipc_contact::FrictionLagged).
using IPCSimplexFrictionLagged = Vector<Float, 10>;

struct IPCSimplexFrictionalContactLaggedLaunchInfo
{
    cuda_tool::CBuffer2DView<ContactCoeff> contact_tabular;
    cuda_tool::CBufferView<IndexT>         contact_element_ids;
    cuda_tool::CBufferView<Vector3>        prev_positions;
    cuda_tool::CBufferView<Vector3>        rest_positions;
    cuda_tool::CBufferView<Float>          thicknesses;
    cuda_tool::CBufferView<Float>          d_hats;

    cuda_tool::CBufferView<Vector4i> PTs;
    cuda_tool::CBufferView<Vector4i> EEs;
    cuda_tool::CBufferView<Vector3i> PEs;
    cuda_tool::CBufferView<Vector2i> PPs;

    cuda_tool::BufferView<IPCSimplexFrictionLagged> lagged;

    Float dt = 0.0;

    // Stream of the launch.
    cudaStream_t stream = cuda_tool::default_stream();
};

// Compute the lagged quantities of every friction pair from the frame-start
// positions (one kernel over the contiguous PT | EE | PE | PP range). The
// energy and assembly launches run it first when asked to (update_lagged).
void launch_ipc_simplex_frictional_contact_lagged(const IPCSimplexFrictionalContactLaggedLaunchInfo& info);

struct IPCSimplexFrictionalContactEnergyLaunchInfo
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

    cuda_tool::BufferView<Float> PT_energies;
    cuda_tool::BufferView<Float> EE_energies;
    cuda_tool::BufferView<Float> PE_energies;
    cuda_tool::BufferView<Float> PP_energies;

    // The pairs' lagged quantities. With update_lagged the launch first
    // recomputes them (launch_ipc_simplex_frictional_contact_lagged);
    // otherwise they come from such a launch for the same pairs and frame.
    cuda_tool::BufferView<IPCSimplexFrictionLagged> lagged;
    bool                                            update_lagged = true;

    Float eps_velocity = 0.0;
    Float dt           = 0.0;

    // Stream of the launch.
    cudaStream_t stream = cuda_tool::default_stream();
};

// Launch one kernel over the contiguous PT | EE | PE | PP range. Empty
// interior ranges are skipped by cumulative offsets; zero total is a no-op.
void launch_ipc_simplex_frictional_contact_energy(const IPCSimplexFrictionalContactEnergyLaunchInfo& info);
}  // namespace uipc::backend::cuda
