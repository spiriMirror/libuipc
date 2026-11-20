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
#include <cuda_tool/buffer.h>
#include <cuda_tool/linear_system/views.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
// These are the production dispatch interfaces, shared by the constitutions
// and regression tests. Empty input is a no-op; gradient_only never uses H.
struct BendingGradientHessianLaunchInfo
{
    cuda_tool::BufferView<Vector4i> stencils;
    cuda_tool::BufferView<Float> stiffnesses, theta_bars, h_bars, volumes, rest_lengths;
    cuda_tool::CBufferView<Vector3>        positions;
    cuda_tool::DoubletVectorView<Float, 3> gradients;
    cuda_tool::TripletMatrixView<Float, 3> hessians;
    Float                                  dt            = 0;
    bool                                   gradient_only = false;
    cuda_tool::BufferView<Float>           yield_stresses;
    cudaStream_t                           stream = nullptr;  // every launch
};
UIPC_BACKEND_API void launch_discrete_shell_bending_gradient_hessian(const BendingGradientHessianLaunchInfo&);
UIPC_BACKEND_API void launch_stress_plastic_bending_gradient_hessian(const BendingGradientHessianLaunchInfo&);

struct NeoHookeanShellGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Float>          lambdas, mus;
    cuda_tool::CBufferView<Vector3i>       indices;
    cuda_tool::CBufferView<Vector3>        positions;
    cuda_tool::CBufferView<Matrix2x2>      inverse_rest_shape_matrices;
    cuda_tool::CBufferView<Float>          thicknesses;
    cuda_tool::DoubletVectorView<Float, 3> gradients;
    cuda_tool::TripletMatrixView<Float, 3> hessians;
    cuda_tool::CBufferView<Float>          rest_areas;
    Float                                  dt            = 0;
    bool                                   gradient_only = false;
    cudaStream_t                           stream = nullptr;  // every launch
};
UIPC_BACKEND_API void launch_neo_hookean_shell_gradient_hessian(const NeoHookeanShellGradientHessianLaunchInfo&);

struct OrthoPotentialGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector12>   qs;
    cuda_tool::CBufferView<Float>      volumes;
    cuda_tool::BufferView<Vector12>    gradients;
    cuda_tool::BufferView<Matrix12x12> hessians;
    cuda_tool::CBufferView<Float>      kappas;
    Float                              dt            = 0;
    bool                               gradient_only = false;
    cudaStream_t                       stream        = nullptr;
};
UIPC_BACKEND_API void launch_ortho_potential_gradient_hessian(const OrthoPotentialGradientHessianLaunchInfo&);
}  // namespace uipc::backend::cuda
