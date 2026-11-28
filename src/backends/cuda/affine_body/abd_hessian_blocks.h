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
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
// Both gathers of ABDLinearSubsystem's 12x12 Hessians into 3x3 triplets run
// one thread per 3x3 block (16 per 12x12 block) instead of one per 12x12
// block, so a few dozen bodies or reporters spread over many SMs. Every
// coefficient sees the arithmetic of the former per-block kernels; the
// triplets and the stored diagonal blocks are bitwise unchanged, and the
// reporters' diagonal contributions are still added atomically.

// Body I's kinetic + shape block (identity for a fixed body) at triplets
// [16 I, 16 I + 16), lower 3x3 blocks zeroed; the full block is stored in
// diag_hessian(I).
UIPC_BACKEND_API void launch_abd_kinetic_shape_hessian_blocks(
    cuda_tool::TripletMatrixView<Float, 3> dst,
    cuda_tool::CBufferView<IndexT>         is_fixed,
    cuda_tool::CBufferView<IndexT>         is_external_kinetic,
    cuda_tool::CBufferView<Matrix12x12>    shape_hessian,
    cuda_tool::CBufferView<Matrix12x12>    kinetic_hessian,
    cuda_tool::BufferView<Matrix12x12>     diag_hessian,
    cudaStream_t                           stream);

// Reporter block I at triplets [16 I, 16 I + 16): zero for a fixed body or a
// lower body pair, lower 3x3 blocks zeroed on the diagonal; a diagonal block
// of a free body is added to diag_hessian.
UIPC_BACKEND_API void launch_abd_reporter_hessian_blocks(
    cuda_tool::TripletMatrixView<Float, 3>       dst,
    cuda_tool::CTripletMatrixView<Float, 12, 12> src,
    cuda_tool::BufferView<Matrix12x12>           diag_hessian,
    cuda_tool::CBufferView<IndexT>               is_fixed,
    cudaStream_t                                 stream);
}  // namespace uipc::backend::cuda
