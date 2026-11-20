/*
 * NVIDIA contributions: interfaces for fused sparse-matrix/vector
 * multiplication and dot products.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
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

#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
// calculate y = a * A * x + b * y
class Spmv
{
  public:
    // symmetric bcoo spmv
    void sym_spmv(Float                                a,
                  cuda_tool::CBCOOMatrixView<Float, 3> A,
                  cuda_tool::CDenseVectorView<Float>   x,
                  Float                                b,
                  cuda_tool::DenseVectorView<Float>    y);

    // reduce by key spmv
    void rbk_spmv(Float                                a,
                  cuda_tool::CBCOOMatrixView<Float, 3> A,
                  cuda_tool::CDenseVectorView<Float>   x,
                  Float                                b,
                  cuda_tool::DenseVectorView<Float>    y);

    // reduce by key symmtric spmv
    void rbk_sym_spmv(Float                                a,
                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                      cuda_tool::CDenseVectorView<Float>   x,
                      Float                                b,
                      cuda_tool::DenseVectorView<Float>    y);

    // reduce by key symmetric spmv with fused dot product
    // computes y = a * A * x  AND  d_dot = x^T * (a * A * x) in a single pass
    // `stream` defaults to the legacy default stream; pass a capture stream
    // when recording a CUDA graph.
    // The triplet count is read on device (`d_triplet_count`) and the grid is
    // sized by `triplet_capacity`, so a captured graph stays valid while the
    // matrix nnz varies within the reserved capacity.
    void rbk_sym_spmv_dot(Float                                a,
                          cuda_tool::CBCOOMatrixView<Float, 3> A,
                          cuda_tool::CDenseVectorView<Float>   x,
                          Float                                b,
                          cuda_tool::DenseVectorView<Float>    y,
                          cuda_tool::VarView<Float>            d_dot,
                          cuda_tool::CDense<IndexT>            d_triplet_count,
                          SizeT                                triplet_capacity,
                          cudaStream_t stream = nullptr);

    // Graph-only ping-pong variant. `y` and `d_dot` must already be zero;
    // while producing them, the same kernel clears the buffers consumed by
    // the next iteration. A converged block becomes a uniform no-op.
    void rbk_sym_spmv_dot_pipelined(Float                                a,
                                    cuda_tool::CBCOOMatrixView<Float, 3> A,
                                    cuda_tool::CDenseVectorView<Float>   x,
                                    cuda_tool::DenseVectorView<Float>    y,
                                    cuda_tool::VarView<Float>            d_dot,
                                    cuda_tool::DenseVectorView<Float>    next_y,
                                    cuda_tool::VarView<Float>   next_dot,
                                    cuda_tool::CVarView<IndexT> converged,
                                    cuda_tool::CDense<IndexT>   d_triplet_count,
                                    SizeT        triplet_capacity,
                                    cudaStream_t stream);

    // debug fallback cpu spmv
    // very slow, only for debug
    void cpu_sym_spmv(Float                                a,
                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                      cuda_tool::CDenseVectorView<Float>   x,
                      Float                                b,
                      cuda_tool::DenseVectorView<Float>    y);
};
}  // namespace uipc::backend::cuda
