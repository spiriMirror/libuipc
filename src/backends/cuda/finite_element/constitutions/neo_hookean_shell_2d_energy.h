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

#include <cuda_tool/buffer.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
struct NeoHookeanShell2DEnergyLaunchInfo
{
    cuda_tool::CBufferView<Float>     lambdas;
    cuda_tool::CBufferView<Float>     mus;
    cuda_tool::CBufferView<Float>     rest_areas;
    cuda_tool::CBufferView<Float>     thicknesses;
    cuda_tool::BufferView<Float>      energies;
    cuda_tool::CBufferView<Vector3i>  indices;
    cuda_tool::CBufferView<Vector3>   positions;
    cuda_tool::CBufferView<Matrix2x2> inverse_rest_shape_matrices;
    Float                             dt = 0.0;
};

// Production launch path for Neo-Hookean shell energy. A zero-sized indices
// view is a no-op.
void launch_neo_hookean_shell_2d_energy(const NeoHookeanShell2DEnergyLaunchInfo& info);
}  // namespace uipc::backend::cuda
