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

#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
// One warp per 12x12 ABD diagonal block.
constexpr int ABDDiagInverseThreads = 32;

// diag_inv(i) = cuda_tool::eigen::inverse(diag_hessian(i)) for every body,
// bitwise identical to the serial GaussEliminationInverse: one warp performs
// the same elimination with lane c owning column c of the augmented [H | I].
void launch_abd_diag_inverse(cuda_tool::CBufferView<Matrix12x12> diag_hessian,
                             cuda_tool::BufferView<Matrix12x12>  diag_inv,
                             cudaStream_t                        stream);
}  // namespace uipc::backend::cuda
