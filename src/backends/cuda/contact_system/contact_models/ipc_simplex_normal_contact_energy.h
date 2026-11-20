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
struct IPCSimplexNormalContactEnergyLaunchInfo
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

    cuda_tool::BufferView<Float> PT_energies;
    cuda_tool::BufferView<Float> EE_energies;
    cuda_tool::BufferView<Float> PE_energies;
    cuda_tool::BufferView<Float> PP_energies;

    Float dt = 0.0;
};

// Launch one kernel over the contiguous PT | EE | PE | PP range. Empty
// interior ranges are skipped by cumulative offsets; zero total is a no-op.
void launch_ipc_simplex_normal_contact_energy(const IPCSimplexNormalContactEnergyLaunchInfo& info);
}  // namespace uipc::backend::cuda
