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
#include <cuda_tool/graph.h>
#include <cub/block/block_reduce.cuh>

namespace uipc::backend::cuda
{
// Block-diagonal fused update/apply/dot steps run on 32-thread blocks. Each
// block updates x and r with alpha = rz / pAp, writes z = inv * r for its
// rows and adds one partial r^T z to rz_new. The per-subsystem kernels and
// the combined launch share these bodies.
constexpr int FusedPcgBlockDiagThreads = 32;

// One block per 12-DoF affine body; lanes 0..11 own the body's rows.
__device__ inline void fused_pcg_block12_update_apply_dot(
    int                                 body,
    cuda_tool::CBufferView<Matrix12x12> diag_inv,
    cuda_tool::DenseVectorView<Float>   x,
    cuda_tool::CDenseVectorView<Float>  p,
    cuda_tool::DenseVectorView<Float>   r,
    cuda_tool::CDenseVectorView<Float>  Ap,
    cuda_tool::DenseVectorView<Float>   z,
    cuda_tool::CDense<Float>            rz,
    cuda_tool::CDense<Float>            pAp,
    cuda_tool::Dense<Float>             rz_new,
    cuda_tool::CDense<IndexT>           converged)
{
    const int  body_count = (int)diag_inv.size();
    const int  lane       = threadIdx.x & (FusedPcgBlockDiagThreads - 1);
    const bool running    = *converged == 0;
    const bool valid      = running && body < body_count && lane < 12;

    Float r_value = 0;
    if(valid)
    {
        const int   i     = body * 12 + lane;
        const Float alpha = *rz / *pAp;
        x(i) += alpha * p(i);
        r_value = r(i) - alpha * Ap(i);
        r(i)    = r_value;
    }

    Float z_value = 0;
#pragma unroll
    for(int col = 0; col < 12; ++col)
    {
        const Float r_col = __shfl_sync(0xffffffffu, r_value, col);
        if(valid)
            z_value = fma(diag_inv(body)(lane, col), r_col, z_value);
    }
    if(valid)
        z(body * 12 + lane) = z_value;

    Float dot = valid ? r_value * z_value : Float{0};
#pragma unroll
    for(int offset = 16; offset > 0; offset >>= 1)
        dot += __shfl_down_sync(0xffffffffu, dot, offset);
    if(lane == 0 && body < body_count && running && dot != Float{0})
        atomicAdd(rz_new.data(), dot);
}

// One block per 32 consecutive 3-DoF vertices.
__device__ inline void fused_pcg_block3_update_apply_dot(
    int                                block,
    cuda_tool::CBufferView<Matrix3x3>  diag_inv,
    cuda_tool::DenseVectorView<Float>  x,
    cuda_tool::CDenseVectorView<Float> p,
    cuda_tool::DenseVectorView<Float>  r,
    cuda_tool::CDenseVectorView<Float> Ap,
    cuda_tool::DenseVectorView<Float>  z,
    cuda_tool::CDense<Float>           rz,
    cuda_tool::CDense<Float>           pAp,
    cuda_tool::Dense<Float>            rz_new,
    cuda_tool::CDense<IndexT>          converged)
{
    using BlockReduce = cub::BlockReduce<Float, FusedPcgBlockDiagThreads>;
    __shared__ typename BlockReduce::TempStorage storage;

    const int  vertex = block * FusedPcgBlockDiagThreads + threadIdx.x;
    const bool valid  = vertex < (int)diag_inv.size() && *converged == 0;
    Float      dot    = 0;
    if(valid)
    {
        const Float alpha = *rz / *pAp;
        Vector3     r_new;
#pragma unroll
        for(int component = 0; component < 3; ++component)
        {
            const int i = vertex * 3 + component;
            x(i) += alpha * p(i);
            r_new(component) = r(i) - alpha * Ap(i);
            r(i)             = r_new(component);
        }

        const Vector3 z_new = diag_inv(vertex) * r_new;
#pragma unroll
        for(int component = 0; component < 3; ++component)
            z(vertex * 3 + component) = z_new(component);
        dot = r_new.dot(z_new);
    }

    const Float block_dot = BlockReduce(storage).Sum(dot);
    if(threadIdx.x == 0 && block_dot != Float{0})
        atomicAdd(rz_new.data(), block_dot);
}

template <typename Block>
struct FusedPcgBlockDiagSegment
{
    cuda_tool::CBufferView<Block>      diag_inv;
    cuda_tool::DenseVectorView<Float>  x;
    cuda_tool::CDenseVectorView<Float> p;
    cuda_tool::DenseVectorView<Float>  r;
    cuda_tool::CDenseVectorView<Float> Ap;
    cuda_tool::DenseVectorView<Float>  z;
};

// One launch for a 12x12 segment followed by a 3x3 segment of the same
// iteration; either may be empty. Each block computes exactly what the
// corresponding per-subsystem kernel would.
void launch_fused_pcg_block_diag_update_apply_dot(
    const FusedPcgBlockDiagSegment<Matrix12x12>& block12,
    const FusedPcgBlockDiagSegment<Matrix3x3>&   block3,
    cuda_tool::CVarView<Float>                   rz,
    cuda_tool::CVarView<Float>                   pAp,
    cuda_tool::VarView<Float>                    rz_new,
    cuda_tool::CVarView<IndexT>                  converged,
    cudaStream_t                                 stream);

void launch_fused_pcg_publish_update_direction(cuda_tool::DenseVectorView<Float> p,
                                               cuda_tool::CDenseVectorView<Float> z,
                                               cuda_tool::VarView<Float> rz,
                                               cuda_tool::CVarView<Float> rz_accum,
                                               cuda_tool::VarView<Float> published_rz_new,
                                               cuda_tool::VarView<Float> next_rz_accum,
                                               cuda_tool::VarView<Float> beta,
                                               cuda_tool::VarView<IndexT> converged,
                                               cuda_tool::CVarView<Float> rz_tol,
                                               cudaStream_t stream);

#if CUDA_TOOL_GRAPH_WHILE
// WHILE-body direction step: beta = rz_new / rz, p = z + beta p, and the loop
// control. rz and the iteration count are double-buffered by the body slot:
// the step reads rz_cur and iter_cur and publishes rz_new to rz_next and the
// count to iter_next (and iter_out), so every block computes beta and the
// decision from the same scalars itself and only block 0 writes the shared
// state. It publishes Converged (|rz_new| <= rz_tol), Stopped (the
// max_iter_minus_1-th iteration without convergence) or Running, also to
// state_out (next to iter_out, the caller can read both with one copy), and
// sets `handle` to keep the WHILE node going only while Running.
void launch_fused_pcg_while_update_direction(cuda_tool::DenseVectorView<Float> p,
                                             cuda_tool::CDenseVectorView<Float> z,
                                             cuda_tool::CVarView<Float> rz_cur,
                                             cuda_tool::VarView<Float>  rz_next,
                                             cuda_tool::CVarView<Float> rz_accum,
                                             cuda_tool::VarView<Float> published_rz_new,
                                             cuda_tool::VarView<Float> next_rz_accum,
                                             cuda_tool::VarView<Float> beta,
                                             cuda_tool::VarView<IndexT> converged,
                                             cuda_tool::CVarView<Float> rz_tol,
                                             cuda_tool::CVarView<IndexT> iter_cur,
                                             cuda_tool::VarView<IndexT> iter_next,
                                             cuda_tool::VarView<IndexT> iter_out,
                                             cuda_tool::VarView<IndexT> state_out,
                                             int max_iter_minus_1,
                                             cudaGraphConditionalHandle handle,
                                             cudaStream_t               stream);
#endif

void launch_fused_pcg_abd_update_apply_dot(cuda_tool::CBufferView<Matrix12x12> diag_inv,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           cuda_tool::DenseVectorView<Float> z,
                                           cuda_tool::CVarView<Float>        rz,
                                           cuda_tool::CVarView<Float> pAp,
                                           cuda_tool::VarView<Float>  rz_new,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream);

void launch_fused_pcg_fem_update_apply_dot(cuda_tool::CBufferView<Matrix3x3> diag_inv,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           cuda_tool::DenseVectorView<Float> z,
                                           cuda_tool::CVarView<Float>        rz,
                                           cuda_tool::CVarView<Float> pAp,
                                           cuda_tool::VarView<Float>  rz_new,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream);
}  // namespace uipc::backend::cuda
