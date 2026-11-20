/*
 * NVIDIA contributions: blockwise ABD Hessian emission; side-stream and
 * prelaunched assembly of the kinetic, shape and reporter producers, Hessian
 * gathers with one thread per 3x3 block, device-side run counts with batched
 * read-backs for the DyTopo body-pair aggregation, and the body-pair
 * aggregation in one block for small entry counts.
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

#include <affine_body/abd_linear_subsystem.h>
#include <affine_body/abd_hessian_blocks.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <cuda_tool/cub.h>
#include <cub/block/block_radix_sort.cuh>
#include <cub/block/block_scan.cuh>
#include <cuda_tool/cuda_tool.h>
#include <utils/matrix_assembler.h>
#include <utils/matrix_unpacker.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constitution_manager.h>
#include <affine_body/abd_linear_subsystem_reporter.h>
#include <affine_body/affine_body_kinetic.h>
#include <affine_body/affine_body_constitution.h>
#include <utils/report_extent_check.h>
#include <array>
#include <cstdint>
#include <limits>

namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;  // for eigen:: / view aliases used inside kernel bodies

    template <typename T>
    void loose_resize(cuda_tool::DeviceBuffer<T>& buffer, SizeT size, Float reserve_ratio)
    {
        if(buffer.capacity() < size)
            buffer.reserve_discard(static_cast<SizeT>(size * reserve_ratio));
        buffer.resize_discard(size);
    }

    inline UIPC_GENERIC std::uint64_t pack_body_pair(IndexT left, IndexT right, int col_bits)
    {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(left)) << col_bits)
               | static_cast<std::uint32_t>(right);
    }

    inline UIPC_GENERIC IndexT body_pair_left(std::uint64_t key, int col_bits)
    {
        return static_cast<IndexT>(key >> col_bits);
    }

    inline UIPC_GENERIC IndexT body_pair_right(std::uint64_t key, int col_bits)
    {
        const auto mask = (std::uint64_t{1} << col_bits) - std::uint64_t{1};
        return static_cast<IndexT>(key & mask);
    }

    __global__ void abd_dytopo_build_body_pair_keys_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> hessians,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::BufferView<std::uint64_t>       keys,
        cuda_tool::BufferView<IndexT>              receiver_indices,
        IndexT                                     vertex_offset,
        int                                        body_pair_col_bits,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        const auto& [g_i, g_j, H3x3] = hessians(I);
        const auto i                 = g_i - vertex_offset;
        const auto j                 = g_j - vertex_offset;
        const auto body_i            = v2b(i);
        const auto body_j            = v2b(j);
        const auto left              = min(body_i, body_j);
        const auto right             = max(body_i, body_j);

        UIPC_KERNEL_ASSERT(left >= 0 && right >= left,
                           "ABD DyTopo body pair must be ordered and nonnegative");
        keys(I)             = pack_body_pair(left, right, body_pair_col_bits);
        receiver_indices(I) = I;
        (void)H3x3;
    }

    // Runs over every receiver slot; the slots at and past the run count
    // (still on the device) get zero pair and block counts, so that the
    // scans may cover all slots.
    __global__ void abd_dytopo_body_pair_block_counts_kernel(
        cuda_tool::CBufferView<std::uint64_t> unique_keys,
        cuda_tool::BufferView<int>            pair_counts,
        cuda_tool::BufferView<int>            block_counts,
        cuda_tool::CDense<int>                pair_count,
        int                                   body_pair_col_bits,
        int                                   n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        if(I >= *pair_count)
        {
            pair_counts(I)  = 0;
            block_counts(I) = 0;
            return;
        }
        const auto key = unique_keys(I);
        // An off-diagonal 12x12 body pair contributes all sixteen 3x3
        // blocks. A diagonal body pair contributes the ten upper blocks;
        // the lower six were discarded by ge2sym in the legacy path.
        block_counts(I) = body_pair_left(key, body_pair_col_bits)
                                  == body_pair_right(key, body_pair_col_bits) ?
                              10 :
                              16;
    }

    // counts = {pair count, total block count}
    __global__ void abd_dytopo_body_pair_total_block_count_kernel(
        cuda_tool::CBufferView<int> block_counts,
        cuda_tool::CBufferView<int> block_offsets,
        cuda_tool::CDense<int>      pair_count,
        cuda_tool::BufferView<int>  counts,
        int                         n)
    {
        if(blockIdx.x == 0 && threadIdx.x == 0)
        {
            counts(0) = *pair_count;
            counts(1) = n == 0 ? 0 : block_offsets(n - 1) + block_counts(n - 1);
        }
    }

    // The whole aggregation in one block for up to ABDBodyPairOneBlockMax
    // receiver entries (a few contacting body pairs): the
    // keys, a stable radix sort of the pair keys (bits [0, 2 col_bits), as
    // DeviceRadixSort::SortPairs), the runs, their counts and offsets, the
    // block counts and offsets, and the two totals. Every output is the same
    // integer the multi-launch path computes.
    constexpr int ABDBodyPairThreads = 1024;
    constexpr int ABDBodyPairItems   = 3;
    constexpr int ABDBodyPairOneBlockMax = ABDBodyPairThreads * ABDBodyPairItems;

    __global__ void __launch_bounds__(ABDBodyPairThreads) abd_dytopo_body_pairs_one_block_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> hessians,
        cuda_tool::CBufferView<IndexT>             v2b,
        IndexT                                     vertex_offset,
        int                                        col_bits,
        cuda_tool::BufferView<IndexT>              receiver_indices,
        cuda_tool::BufferView<std::uint64_t>       unique_keys,
        cuda_tool::BufferView<int>                 pair_counts,
        cuda_tool::BufferView<int>                 pair_offsets,
        cuda_tool::BufferView<int>                 block_counts,
        cuda_tool::BufferView<int>                 block_offsets,
        cuda_tool::BufferView<int>                 counts,
        int                                        n)
    {
        using Sort =
            cub::BlockRadixSort<std::uint64_t, ABDBodyPairThreads, ABDBodyPairItems, int>;
        using Scan = cub::BlockScan<int, ABDBodyPairThreads>;
        // the sort's storage is free once the keys are back in registers
        __shared__ union
        {
            typename Sort::TempStorage sort;
            struct
            {
                std::uint64_t sorted[ABDBodyPairOneBlockMax];
                int           starts[ABDBodyPairOneBlockMax];
            } runs;
        } temp;
        __shared__ typename Scan::TempStorage scan_temp;
        __shared__ int                        run_total;
        auto&                                 sorted = temp.runs.sorted;
        auto&                                 starts = temp.runs.starts;

        std::uint64_t keys[ABDBodyPairItems];
        int           order[ABDBodyPairItems];
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
        {
            const int I = threadIdx.x * ABDBodyPairItems + k;
            order[k]    = I;
            keys[k]     = ~std::uint64_t{0};  // after every entry (stable)
            if(I < n)
            {
                const auto& [g_i, g_j, H3x3] = hessians(I);
                const auto body_i            = v2b(g_i - vertex_offset);
                const auto body_j            = v2b(g_j - vertex_offset);
                const auto left              = min(body_i, body_j);
                const auto right             = max(body_i, body_j);
                UIPC_KERNEL_ASSERT(left >= 0 && right >= left,
                                   "ABD DyTopo body pair must be ordered and nonnegative");
                keys[k] = pack_body_pair(left, right, col_bits);
                (void)H3x3;
            }
        }
        Sort(temp.sort).Sort(keys, order, 0, 2 * col_bits);
        __syncthreads();

        int heads[ABDBodyPairItems];
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
            sorted[threadIdx.x * ABDBodyPairItems + k] = keys[k];
        __syncthreads();
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
        {
            const int p = threadIdx.x * ABDBodyPairItems + k;
            if(p < n)
                receiver_indices(p) = order[k];
            heads[k] = p < n && (p == 0 || sorted[p] != sorted[p - 1]) ? 1 : 0;
        }
        int runs[ABDBodyPairItems];
        int run_count;
        Scan(scan_temp).ExclusiveSum(heads, runs, run_count);
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
            if(heads[k])
                starts[runs[k]] = threadIdx.x * ABDBodyPairItems + k;
        if(threadIdx.x == 0)
            run_total = run_count;
        __syncthreads();

        // per run: key, entry count and offset, block count; then the block offsets
        int block[ABDBodyPairItems];
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
        {
            const int r = threadIdx.x * ABDBodyPairItems + k;
            block[k]    = 0;
            if(r < run_total)
            {
                const int  start = starts[r];
                const int  end   = r + 1 < run_total ? starts[r + 1] : n;
                const auto key   = sorted[start];
                unique_keys(r)   = key;
                pair_offsets(r)  = start;
                pair_counts(r)   = end - start;
                block[k] =
                    body_pair_left(key, col_bits) == body_pair_right(key, col_bits) ? 10 : 16;
                block_counts(r) = block[k];
            }
        }
        __syncthreads();
        int block_offset[ABDBodyPairItems];
        int block_total;
        Scan(scan_temp).ExclusiveSum(block, block_offset, block_total);
#pragma unroll
        for(int k = 0; k < ABDBodyPairItems; ++k)
        {
            const int r = threadIdx.x * ABDBodyPairItems + k;
            if(r < run_total)
                block_offsets(r) = block_offset[k];
        }
        if(threadIdx.x == 0)
        {
            counts(0) = run_total;
            counts(1) = block_total;
        }
    }

    inline UIPC_GENERIC int same_body_upper_block_slot(int block_i, int block_j)
    {
        // Row starts are 0, 4, 7, 9 for the upper triangle of a 4x4
        // block matrix.
        return block_i * 3 - block_i * (block_i - 1) / 2 + block_j;
    }

    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k1_kernel(
        cuda_tool::CBufferView<IndexT>    is_fixed,
        cuda_tool::CBufferView<IndexT>    is_external_kinetic,
        cuda_tool::CBufferView<Vector12>  shape_gradient,
        cuda_tool::CBufferView<Vector12>  kinetic_gradient,
        cuda_tool::DenseVectorView<Float> gradients,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector12 src;

        if(is_fixed(i))
        {
            src.setZero();  // if fixed, set to zero
        }
        else
        {
            src = shape_gradient(i);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(i)) [[likely]]
            {
                src += kinetic_gradient(i);
            }
        }

        gradients.segment<12>(i * 12) = src;

        // cout << "EKG(" << i << "): " << src.transpose().eval() << "\n";
    }

    // One thread per 3x3 block (t = 16 I + 4 ii + jj) of body I's 12x12
    // kinetic + shape Hessian.
    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3> dst,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        cuda_tool::CBufferView<IndexT>         is_external_kinetic,
        cuda_tool::CBufferView<Matrix12x12>    shape_hessian,
        cuda_tool::CBufferView<Matrix12x12>    kinetic_hessian,
        cuda_tool::BufferView<Matrix12x12>     diag_hessian,
        int                                    n)
    {
        int t = blockIdx.x * blockDim.x + threadIdx.x;
        if(t >= n)
            return;
        const int I  = t / 16;
        const int ii = (t % 16) / 4;
        const int jj = t % 4;
        Matrix3x3 H;

        if(is_fixed(I))
        {
            // Fill kinetic hessian to identity to avoid singularity
            if(ii == jj)
                H.setIdentity();
            else
                H.setZero();
        }
        else
        {
            // if not fixed, fill shape hessian
            H = shape_hessian(I).block<3, 3>(ii * 3, jj * 3);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(I)) [[likely]]
            {
                H += kinetic_hessian(I).block<3, 3>(ii * 3, jj * 3);
            }
        }

        // record diagonal hessian for diag-inv preconditioner
        diag_hessian(I).block<3, 3>(ii * 3, jj * 3) = H;

        // set the lower triangle blocks to zero for robustness
        if(ii > jj)
            H.setZero();

        dst(t).write(I * 4 + ii, I * 4 + jj, H);
    }

    __global__ void abd_linear_subsystem_assemble_reporters_k1_kernel(
        cuda_tool::DenseVectorView<Float>        dst,
        cuda_tool::CDoubletVectorView<Float, 12> src,
        cuda_tool::CBufferView<IndexT>           is_fixed,
        int                                      n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [body_i, G12] = src(I);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            dst.segment<12>(body_i * 12).atomic_add(G12);
        }
    }

    // One thread per 3x3 block (t = 16 I + 4 ii + jj) of reporter block I.
    __global__ void abd_linear_subsystem_assemble_reporters_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3>       dst,
        cuda_tool::CTripletMatrixView<Float, 12, 12> src,
        cuda_tool::BufferView<Matrix12x12>           diag_hessian,
        cuda_tool::CBufferView<IndexT>               is_fixed,
        int                                          n)
    {
        int t = blockIdx.x * blockDim.x + threadIdx.x;
        if(t >= n)
            return;
        const int I  = t / 16;
        const int ii = (t % 16) / 4;
        const int jj = t % 4;

        auto&& [body_i, body_j, Value] = src(I);
        Matrix3x3 H                    = Value.block<3, 3>(ii * 3, jj * 3);

        bool has_fixed = (is_fixed(body_i) || is_fixed(body_j));

        // Fill diagonal hessian for diag-inv preconditioner
        if(body_i == body_j && !has_fixed)
        {
            auto& D = diag_hessian(body_i);
            for(int c = 0; c < 3; ++c)
                for(int r = 0; r < 3; ++r)
                    cuda_tool::atomic_add(&D(ii * 3 + r, jj * 3 + c), H(r, c));
        }

        if(has_fixed)
        {
            // Zero out hessian for fixed bodies
            H.setZero();
        }
        else
        {
            if(body_i == body_j)
            {
                // Since body_i == body_j, we only fill the upper triangle part
                if(ii > jj)
                    H.setZero();
            }
            else if(body_i > body_j)
            {
                // If all the reporters only report upper triangle part, this branch should not be hit
                H.setZero();
            }
        }

        dst(t).write(body_i * 4 + ii, body_j * 4 + jj, H);
    }

    __global__ void abd_linear_subsystem_assemble_dytopo_effect_k1_kernel(
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        cuda_tool::DenseVectorView<Float>       gradients,
        cuda_tool::CBufferView<IndexT>          v2b,
        cuda_tool::CBufferView<ABDJacobi>       Js,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        IndexT                                  vertex_offset,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, G3] = dytopo_effect_gradient(I);

        auto  i      = g_i - vertex_offset;
        auto  body_i = v2b(i);
        auto& J_i    = Js(i);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            Vector12 G12 = J_i.T() * G3;
            gradients.segment<12>(body_i * 12).atomic_add(G12);

            // cout << "DG(" << I << "): " << G12.transpose().eval() << "\n";
        }
    }

    // One CUDA block owns one ordered body pair. Its sixteen warps each
    // reduce one 3x3 block of the corresponding 12x12 body-pair matrix.
    // Same-body lower blocks are retained long enough to update the complete
    // diagonal Hessian, but only the ten upper blocks enter the global
    // triplet stream, matching the legacy ge2sym contract.
    __global__ void abd_linear_subsystem_aggregate_dytopo_body_pairs_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3>     dst,
        cuda_tool::CBufferView<IndexT>             sorted_receiver_indices,
        cuda_tool::CBufferView<std::uint64_t>      unique_body_pair_keys,
        cuda_tool::CBufferView<int>                body_pair_counts,
        cuda_tool::CBufferView<int>                body_pair_offsets,
        cuda_tool::CBufferView<int>                body_pair_block_offsets,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::CBufferView<ABDJacobi>          Js,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        cuda_tool::BufferView<Matrix12x12>         diag_hessian,
        IndexT                                     vertex_offset,
        int                                        body_pair_col_bits,
        int                                        pair_count)
    {
        constexpr int warp_size = 32;

        const int pair_index  = blockIdx.x;
        const int block_index = threadIdx.x / warp_size;
        const int lane_index  = threadIdx.x & (warp_size - 1);
        if(pair_index >= pair_count || block_index >= 16)
            return;

        const auto pair_key  = unique_body_pair_keys(pair_index);
        const auto left      = body_pair_left(pair_key, body_pair_col_bits);
        const auto right     = body_pair_right(pair_key, body_pair_col_bits);
        const auto block_i   = block_index / 4;
        const auto block_j   = block_index % 4;
        const bool same_body = left == right;
        const bool has_fixed = is_fixed(left) || is_fixed(right);

        Matrix3x3 sum = Matrix3x3::Zero();
        if(!has_fixed)
        {
            const int pair_offset = body_pair_offsets(pair_index);
            const int pair_size   = body_pair_counts(pair_index);
            for(int k = lane_index; k < pair_size; k += warp_size)
            {
                const auto receiver_index = sorted_receiver_indices(pair_offset + k);
                const auto& [g_i, g_j, H3x3] = dytopo_effect_hessian(receiver_index);

                const auto i      = g_i - vertex_offset;
                const auto j      = g_j - vertex_offset;
                const auto body_i = v2b(i);
                const auto body_j = v2b(j);
                Matrix3x3  block;

                if(body_i < body_j)
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j);
                }
                else if(body_i > body_j)
                {
                    block = ABDJacobi::JT_H_J_block(
                        Js(j).T(), H3x3.transpose(), Js(i), block_i, block_j);
                }
                else if(i != j)
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j)
                            + ABDJacobi::JT_H_J_block(
                                Js(j).T(), H3x3.transpose(), Js(i), block_i, block_j);
                }
                else
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j);
                }
                sum += block;
            }

#pragma unroll
            for(int delta = warp_size / 2; delta > 0; delta /= 2)
            {
#pragma unroll
                for(int row = 0; row < 3; ++row)
                {
#pragma unroll
                    for(int col = 0; col < 3; ++col)
                    {
                        sum(row, col) +=
                            __shfl_down_sync(0xFFFFFFFFu, sum(row, col), delta);
                    }
                }
            }
        }

        if(lane_index != 0)
            return;

        if(same_body)
        {
            if(!has_fixed)
            {
#pragma unroll
                for(int row = 0; row < 3; ++row)
                {
#pragma unroll
                    for(int col = 0; col < 3; ++col)
                    {
                        diag_hessian(left)(3 * block_i + row, 3 * block_j + col) +=
                            sum(row, col);
                    }
                }
            }

            if(block_i > block_j)
                return;

            const int output_index = body_pair_block_offsets(pair_index)
                                     + same_body_upper_block_slot(block_i, block_j);
            dst(output_index).write(left * 4 + block_i, right * 4 + block_j, sum);
        }
        else
        {
            const int output_index = body_pair_block_offsets(pair_index) + block_index;
            dst(output_index).write(left * 4 + block_i, right * 4 + block_j, sum);
        }
    }

    __global__ void abd_linear_subsystem_retrieve_solution_kernel(
        cuda_tool::BufferView<Vector12> dq, cuda_tool::CDenseVectorView<Float> x, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // retrieve solution for each body
        dq(i) = -x.segment<12>(i * 12).as_eigen();
    }

    __global__ void abd_linear_subsystem_diag_norm_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_hess,
        cuda_tool::BufferView<Float>        diag_blocks_norm,
        cuda_tool::CBufferView<IndexT>      is_fixed,
        int                                 n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        for(int i = 0; i < 12; i++)
            diag_blocks_norm(idx * 12 + i) =
                is_fixed(idx) ? 0 : abs(diag_hess(idx)(i, i));
    }

    __global__ void abd_linear_subsystem_mass_norm_kernel(
        cuda_tool::CBufferView<ABDJacobiDyadicMass> mass,
        cuda_tool::BufferView<Float>                block_norm,
        cuda_tool::CBufferView<IndexT>              is_fixed,
        int                                         n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        block_norm(idx) = is_fixed(idx) ? 0 : mass(idx).mass();
    }
}  // namespace
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
// 16 threads per 12x12 block in blocks of 64 threads: a few dozen bodies or
// reporters spread over many SMs instead of one block on one SM.
constexpr int ABDHessianBlockThreads = 64;

void launch_abd_kinetic_shape_hessian_blocks(cuda_tool::TripletMatrixView<Float, 3> dst,
                                             cuda_tool::CBufferView<IndexT> is_fixed,
                                             cuda_tool::CBufferView<IndexT> is_external_kinetic,
                                             cuda_tool::CBufferView<Matrix12x12> shape_hessian,
                                             cuda_tool::CBufferView<Matrix12x12> kinetic_hessian,
                                             cuda_tool::BufferView<Matrix12x12> diag_hessian,
                                             cudaStream_t stream)
{
    int n = (int)shape_hessian.size() * 16;
    if(n > 0)
        abd_linear_subsystem_assemble_kinetic_shape_k2_kernel<<<(n + ABDHessianBlockThreads - 1) / ABDHessianBlockThreads, ABDHessianBlockThreads, 0, stream>>>(
            dst, is_fixed, is_external_kinetic, shape_hessian, kinetic_hessian, diag_hessian, n);
}

void launch_abd_reporter_hessian_blocks(cuda_tool::TripletMatrixView<Float, 3> dst,
                                        cuda_tool::CTripletMatrixView<Float, 12, 12> src,
                                        cuda_tool::BufferView<Matrix12x12> diag_hessian,
                                        cuda_tool::CBufferView<IndexT> is_fixed,
                                        cudaStream_t                   stream)
{
    int n = src.triplet_count() * 16;
    if(n > 0)
        abd_linear_subsystem_assemble_reporters_k2_kernel<<<(n + ABDHessianBlockThreads - 1) / ABDHessianBlockThreads, ABDHessianBlockThreads, 0, stream>>>(
            dst, src, diag_hessian, is_fixed, n);
}

REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

// ref: https://github.com/spiriMirror/libuipc/issues/271
constexpr U64 ABDLinearSubsystemUID = 0ull;

void ABDLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo& info)
{
    m_impl.affine_body_dynamics        = require<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = require<AffineBodyVertexReporter>();
    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");

    m_impl.dytopo_effect_receiver = find<ABDDyTopoEffectReceiver>();
}

void ABDLinearSubsystem::Impl::init()
{
    auto reporter_view = reporters.view();
    for(auto&& [i, r] : enumerate(reporter_view))
        r->m_index = i;
    for(auto& r : reporter_view)
        r->init();

    reporter_gradient_offsets_counts.resize(reporter_view.size());
    reporter_hessian_offsets_counts.resize(reporter_view.size());

    SizeT body_count = abd().body_count();
    body_id_to_shape_hessian.resize(body_count);
    body_id_to_shape_gradient.resize(body_count);
    body_id_to_kinetic_hessian.resize(body_count);
    body_id_to_kinetic_gradient.resize(body_count);
    diag_hessian.resize(body_count);
}

void ABDLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(abd().body_count() * 12);
}

void ABDLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = abd().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    // fill the dof_offset and dof_count for each geometry
    affine_body_dynamics->for_each(
        geo_slots,
        [&](const AffineBodyDynamics::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on ABD mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on ABD mesh why can it happen?");

            IndexT this_dof_count = 12 * sc.instances().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

SizeT ABDLinearSubsystem::Impl::_prepare_dytopo_body_pair_aggregation()
{
    using namespace cuda_tool;

    const SizeT receiver_count =
        dytopo_effect_receiver ? dytopo_effect_receiver->hessians().triplet_count() : 0;
    dytopo_prepared_receiver_count = receiver_count;
    dytopo_unique_body_pair_count  = 0;
    dytopo_aggregated_block_count  = 0;

    if(receiver_count == 0)
        return 0;

    UIPC_ASSERT(receiver_count <= static_cast<SizeT>(std::numeric_limits<int>::max()),
                "ABD DyTopo receiver count {} exceeds the CUDA kernel index range",
                receiver_count);
    UIPC_ASSERT(abd().body_count()
                    <= static_cast<SizeT>(std::numeric_limits<IndexT>::max()),
                "ABD body count {} exceeds the IndexT range",
                abd().body_count());

    std::uint32_t max_body_id =
        abd().body_count() > 0 ? static_cast<std::uint32_t>(abd().body_count() - 1) : 0;
    int col_bits = 1;
    while(max_body_id >>= 1)
        ++col_bits;
    dytopo_body_pair_col_bits = col_bits;

    if(receiver_count <= static_cast<SizeT>(ABDBodyPairOneBlockMax))
    {
        loose_resize(dytopo_receiver_indices, receiver_count, reserve_ratio);
        loose_resize(dytopo_unique_body_pair_keys, receiver_count, reserve_ratio);
        loose_resize(dytopo_body_pair_counts, receiver_count, reserve_ratio);
        loose_resize(dytopo_body_pair_offsets, receiver_count, reserve_ratio);
        loose_resize(dytopo_body_pair_block_counts, receiver_count, reserve_ratio);
        loose_resize(dytopo_body_pair_block_offsets, receiver_count, reserve_ratio);
        dytopo_body_pair_count_readback.resize_discard(2);
        abd_dytopo_body_pairs_one_block_kernel<<<1, ABDBodyPairThreads>>>(
            dytopo_effect_receiver->hessians(),
            abd().vertex_id_to_body_id.cview(),
            affine_body_vertex_reporter->vertex_offset(),
            dytopo_body_pair_col_bits,
            dytopo_receiver_indices.view(),
            dytopo_unique_body_pair_keys.view(),
            dytopo_body_pair_counts.view(),
            dytopo_body_pair_offsets.view(),
            dytopo_body_pair_block_counts.view(),
            dytopo_body_pair_block_offsets.view(),
            dytopo_body_pair_count_readback.view(),
            static_cast<int>(receiver_count));
        return _finish_dytopo_body_pair_aggregation();
    }

    loose_resize(dytopo_body_pair_keys_input, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_keys, receiver_count, reserve_ratio);
    loose_resize(dytopo_receiver_indices_input, receiver_count, reserve_ratio);
    loose_resize(dytopo_receiver_indices, receiver_count, reserve_ratio);

    auto k = abd_dytopo_build_body_pair_keys_kernel;
    int  n = static_cast<int>(receiver_count);
    k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        dytopo_effect_receiver->hessians(),
        abd().vertex_id_to_body_id.cview(),
        dytopo_body_pair_keys_input.view(),
        dytopo_receiver_indices_input.view(),
        affine_body_vertex_reporter->vertex_offset(),
        dytopo_body_pair_col_bits,
        n);

    DeviceRadixSort().SortPairs(dytopo_body_pair_keys_input.data(),
                                dytopo_body_pair_keys.data(),
                                dytopo_receiver_indices_input.data(),
                                dytopo_receiver_indices.data(),
                                static_cast<int>(receiver_count),
                                0,
                                2 * dytopo_body_pair_col_bits);

    loose_resize(dytopo_unique_body_pair_keys, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_counts, receiver_count, reserve_ratio);
    DeviceRunLengthEncode().Encode(dytopo_body_pair_keys.data(),
                                   dytopo_unique_body_pair_keys.data(),
                                   dytopo_body_pair_counts.data(),
                                   dytopo_unique_body_pair_count_dev.data(),
                                   static_cast<int>(receiver_count));

    // The run count stays on the device: the block counts and both scans
    // cover all receiver slots, the slots past the run count holding zeros,
    // so that the run count and the aggregated block count come back in one
    // read-back. The integer prefix sums of the runs do not depend on the
    // slots after them.
    loose_resize(dytopo_body_pair_offsets, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_block_counts, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_block_offsets, receiver_count, reserve_ratio);

    auto k2 = abd_dytopo_body_pair_block_counts_kernel;
    k2<<<cuda_tool::best_grid_dim(n, k2), cuda_tool::best_block_dim(k2), 0, nullptr>>>(
        dytopo_unique_body_pair_keys.cview(),
        dytopo_body_pair_counts.view(),
        dytopo_body_pair_block_counts.view(),
        dytopo_unique_body_pair_count_dev.cviewer(),
        dytopo_body_pair_col_bits,
        n);

    DeviceScan().ExclusiveSum(
        dytopo_body_pair_counts.data(), dytopo_body_pair_offsets.data(), n);
    DeviceScan().ExclusiveSum(dytopo_body_pair_block_counts.data(),
                              dytopo_body_pair_block_offsets.data(),
                              n);
    dytopo_body_pair_count_readback.resize_discard(2);
    abd_dytopo_body_pair_total_block_count_kernel<<<1, 1>>>(
        dytopo_body_pair_block_counts.cview(),
        dytopo_body_pair_block_offsets.cview(),
        dytopo_unique_body_pair_count_dev.cviewer(),
        dytopo_body_pair_count_readback.view(),
        n);
    return _finish_dytopo_body_pair_aggregation();
}

SizeT ABDLinearSubsystem::Impl::_finish_dytopo_body_pair_aggregation()
{
    std::array<int, 2> counts{};
    dytopo_body_pair_count_readback.copy_to(counts.data());
    const int pair_count  = counts[0];
    const int block_count = counts[1];
    UIPC_ASSERT(pair_count >= 0, "ABD DyTopo body-pair run count must be nonnegative");
    UIPC_ASSERT(block_count >= 0, "ABD DyTopo aggregated block count must be nonnegative");
    dytopo_unique_body_pair_count = static_cast<SizeT>(pair_count);
    dytopo_unique_body_pair_keys.resize_discard(pair_count);
    dytopo_body_pair_counts.resize_discard(pair_count);
    dytopo_body_pair_offsets.resize_discard(pair_count);
    dytopo_body_pair_block_counts.resize_discard(pair_count);
    dytopo_body_pair_block_offsets.resize_discard(pair_count);
    dytopo_aggregated_block_count = static_cast<SizeT>(block_count);

    return dytopo_aggregated_block_count;
}

void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    // 1. Gradient Count
    constexpr SizeT G12_to_dof = 12;
    SizeT           body_count = abd().body_count();
    auto            dof_count  = body_count * G12_to_dof;

    auto has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    SizeT H12x12_count = 0;

    if(has_complement)
    {
        // 1) Body hessian: kinetic + shape
        if(!info.gradient_only())
            H12x12_count += abd().body_count();

        // 2) Reporters
        _report_reporter_extents(info.gradient_only());

        if(!info.gradient_only())
            H12x12_count += reporter_hessian_offsets_counts.total_count();
    }


    SizeT dytopo_H3x3_count = 0;
    if(dytopo_effect_receiver && !info.gradient_only())
        dytopo_H3x3_count = _prepare_dytopo_body_pair_aggregation();


    auto H3x3_count = H12x12_count * (4 * 4) + dytopo_H3x3_count;

    if(info.gradient_only())
    {
        UIPC_ASSERT(H3x3_count == 0,
                    "Hessian block count should be zero (got {}) when gradient_only is true",
                    H3x3_count);
    }

    info.extent(H3x3_count, dof_count);
}

void ABDLinearSubsystem::Impl::_report_reporter_extents(bool gradient_only)
{
    auto grad_counts = reporter_gradient_offsets_counts.counts();
    auto hess_counts = reporter_hessian_offsets_counts.counts();

    for(auto&& R : reporters.view())
    {
        ReportExtentInfo extent_info;
        extent_info.m_gradient_only = gradient_only;
        R->report_extent(extent_info);

        grad_counts[R->m_index] = extent_info.m_gradient_count;
        hess_counts[R->m_index] = extent_info.m_hessian_count;
    }

    reporter_gradient_offsets_counts.scan();
    reporter_hessian_offsets_counts.scan();
}

void ABDLinearSubsystem::Impl::_prepare_reporter_buffers()
{
    auto N = abd().body_count();

    reporter_gradients.reshape(N);
    reporter_gradients.resize_doublets_discard(reporter_gradient_offsets_counts.total_count());

    reporter_hessians.reshape(N, N);
    reporter_hessians.resize_triplets_discard(reporter_hessian_offsets_counts.total_count());
}

void ABDLinearSubsystem::Impl::prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
    // Kinetic, shape and reporter terms read only the Newton iterate and
    // write ABD-private buffers, and the reporter extents do not depend on
    // contact, so they can run alongside the DyTopo computation. Side stream
    // 0 carries the kinetic/shape chain that assemble() continues.
    auto side_streams = info.side_streams();
    _report_reporter_extents(false);
    _prepare_reporter_buffers();
    prelaunched_gradient_count = reporter_gradients.doublet_count();
    prelaunched_hessian_count  = reporter_hessians.triplet_count();

    side_streams->fork();
    _compute_kinetic_shape(false, side_streams->next());
    _launch_reporters(false, side_streams);
}

void ABDLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // 0) Prepare buffers for reporters (unchanged sizes after a prelaunch)
    _prepare_reporter_buffers();

    bool has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    auto side_streams = info.side_streams();
    bool prelaunched  = info.prelaunched();
    UIPC_ASSERT(!prelaunched || (side_streams && has_complement && !info.gradient_only()),
                "ABD prelaunch only serves a full forked assembly");
    UIPC_ASSERT(!prelaunched
                    || (prelaunched_gradient_count == reporter_gradients.doublet_count()
                        && prelaunched_hessian_count == reporter_hessians.triplet_count()),
                "ABD reporter extents changed after the prelaunch");

    // Reporters write disjoint ranges of reporter_gradients/hessians and read
    // only state completed before this assembly, so with side streams they
    // run there from this point on, next to the kinetic/shape work of the
    // legacy stream.
    if(side_streams && !prelaunched)
        side_streams->fork();

    // After a prelaunch the kinetic/shape stores continue its chain on side
    // stream 0, once the legacy stream has cleared A and b.
    cudaStream_t kinetic_shape_stream = nullptr;
    if(prelaunched)
    {
        kinetic_shape_stream = (*side_streams)[0];
        side_streams->fork_one(0);
    }

    IndexT hess_offset = 0;

    // 1) Static Topo Effect: Kinetic + Shape + Other Reporters
    if(has_complement)
    {
        if(!prelaunched)
        {
            _compute_kinetic_shape(info.gradient_only(), nullptr);
            _launch_reporters(info.gradient_only(), side_streams);
        }
        _assemble_kinetic_shape(hess_offset, info, kinetic_shape_stream);
    }
    else  // contact only
    {
        info.gradients().buffer_view().fill(0);
    }

    // The gathers add atomically onto the plain stores above and read every
    // reporter range: on side stream 0 they wait for the legacy stream and
    // all reporters, and the global system joins them before the conversion.
    cudaStream_t gather_stream = nullptr;
    if(side_streams && side_streams->size())
    {
        gather_stream = (*side_streams)[0];
        side_streams->fork_one(0);
        side_streams->join(gather_stream);
    }

    if(has_complement)
        _assemble_reporters(hess_offset, info, gather_stream);

    // 2) Dynamic Topology Effect
    _assemble_dytopo_effect(hess_offset, info, gather_stream);

    UIPC_ASSERT(hess_offset == info.hessians().triplet_count(),
                "Hessian size mismatch: expected {}, got {}",
                info.hessians().triplet_count(),
                hess_offset);
}

void ABDLinearSubsystem::Impl::_compute_kinetic_shape(bool gradient_only, cudaStream_t stream)
{
    Float dt = dt_attr->view()[0];

    // Collect Kinetic
    ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
        gradient_only, body_id_to_kinetic_gradient, body_id_to_kinetic_hessian, dt, stream};
    abd().kinetic->compute_gradient_hessian(this_info);

    // Collect Shape
    for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
    {
        ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
            gradient_only,
            abd().subview(body_id_to_shape_gradient, cst->m_index),
            abd().subview(body_id_to_shape_hessian, cst->m_index),
            dt,
            stream};

        cst->compute_gradient_hessian(this_info);
    }
}

void ABDLinearSubsystem::Impl::_assemble_kinetic_shape(IndexT& hess_offset,
                                                       GlobalLinearSystem::DiagInfo& info,
                                                       cudaStream_t stream)
{
    using namespace cuda_tool;

    {
        auto k = abd_linear_subsystem_assemble_kinetic_shape_k1_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
                abd().body_id_to_is_fixed.cview(),
                abd().body_id_to_external_kinetic.cview(),
                body_id_to_shape_gradient.cview(),
                body_id_to_kinetic_gradient.cview(),
                info.gradients(),
                n);
    }

    if(info.gradient_only())
        return;

    auto body_count = body_id_to_shape_hessian.size();
    auto H3x3_count = body_count * (4 * 4);
    auto body_H3x3  = info.hessians().subview(hess_offset, H3x3_count);

    {
        launch_abd_kinetic_shape_hessian_blocks(
            body_H3x3,
            abd().body_id_to_is_fixed.cview(),
            abd().body_id_to_external_kinetic.cview(),
            body_id_to_shape_hessian.cview(),
            body_id_to_kinetic_hessian.cview(),
            diag_hessian.view(),
            stream);
    }

    hess_offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::_launch_reporters(bool gradient_only,
                                                 cuda_tool::SideStreams* side_streams)
{
    // Fill TripletMatrix and DoubletVector
    for(auto& R : reporters.view())
    {
        AssembleInfo assemble_info{this, R->m_index, gradient_only, side_streams};
        R->assemble(assemble_info);
    }
}

void ABDLinearSubsystem::Impl::_assemble_reporters(IndexT& offset,
                                                   GlobalLinearSystem::DiagInfo& info,
                                                   cudaStream_t stream)
{
    using namespace cuda_tool;

    if(reporter_gradients.doublet_count())
    {
        auto k = abd_linear_subsystem_assemble_reporters_k1_kernel;
        int  n = (int)reporter_gradients.doublet_count();
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            info.gradients(),
            reporter_gradients.cview(),
            abd().body_id_to_is_fixed.cview(),
            n);
    }

    if(!info.gradient_only() && reporter_hessians.triplet_count())
    {
        // get rest
        auto H3x3s = info.hessians().subview(offset);

        launch_abd_reporter_hessian_blocks(H3x3s,
                                           reporter_hessians.cview(),
                                           diag_hessian.view(),
                                           abd().body_id_to_is_fixed.cview(),
                                           stream);

        offset += reporter_hessians.triplet_count() * (4 * 4);
    }
}

void ABDLinearSubsystem::Impl::_assemble_dytopo_effect(IndexT& offset,
                                                       GlobalLinearSystem::DiagInfo& info,
                                                       cudaStream_t stream)
{
    using namespace cuda_tool;

    auto  vertex_offset = affine_body_vertex_reporter->vertex_offset();
    SizeT dytopo_effect_gradient_count = 0;
    if(dytopo_effect_receiver)
    {
        dytopo_effect_gradient_count =
            dytopo_effect_receiver->gradients().doublet_count();
    }

    if(dytopo_effect_gradient_count)
    {
        auto k = abd_linear_subsystem_assemble_dytopo_effect_k1_kernel;
        int  n = (int)dytopo_effect_gradient_count;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            dytopo_effect_receiver->gradients(),
            info.gradients(),
            abd().vertex_id_to_body_id.cview(),
            abd().vertex_id_to_J.cview(),
            abd().body_id_to_is_fixed.cview(),
            vertex_offset,
            n);
    }

    if(info.gradient_only())
        return;

    SizeT dytopo_effect_hessian_count = 0;
    if(dytopo_effect_receiver)
        dytopo_effect_hessian_count = dytopo_effect_receiver->hessians().triplet_count();

    if(dytopo_effect_hessian_count)
    {
        UIPC_ASSERT(dytopo_prepared_receiver_count == dytopo_effect_hessian_count,
                    "ABD DyTopo body-pair preparation is stale: prepared {}, assembling {}",
                    dytopo_prepared_receiver_count,
                    dytopo_effect_hessian_count);
    }

    auto H3x3_count         = dytopo_aggregated_block_count;
    auto dytopo_effect_H3x3 = info.hessians().subview(offset, H3x3_count);

    if(dytopo_effect_hessian_count)
    {
        // Half Contact Hessian
        // ref: https://github.com/spiriMirror/libuipc/issues/272
        constexpr int block_dim = 16 * 32;
        const int pair_count = static_cast<int>(dytopo_unique_body_pair_count);
        if(pair_count > 0)
            abd_linear_subsystem_aggregate_dytopo_body_pairs_kernel<<<pair_count, block_dim, 0, stream>>>(
                dytopo_effect_receiver->hessians(),
                dytopo_effect_H3x3,
                dytopo_receiver_indices.cview(),
                dytopo_unique_body_pair_keys.cview(),
                dytopo_body_pair_counts.cview(),
                dytopo_body_pair_offsets.cview(),
                dytopo_body_pair_block_offsets.cview(),
                abd().vertex_id_to_body_id.cview(),
                abd().vertex_id_to_J.cview(),
                abd().body_id_to_is_fixed.cview(),
                diag_hessian.view(),
                vertex_offset,
                dytopo_body_pair_col_bits,
                pair_count);
    }

    offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.satisfied(true);
}

void ABDLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace cuda_tool;

    auto dq = abd().body_id_to_dq.view();
    {
        auto k = abd_linear_subsystem_retrieve_solution_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                dq, info.solution(), n);
    }
}

Float ABDLinearSubsystem::Impl::diag_norm()
{
    auto diag_hess = diag_hessian.view();
    block_norm.resize(diag_hess.size() * 12);
    {
        auto k = abd_linear_subsystem_diag_norm_kernel;
        int  n = (int)diag_hess.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                diag_hess.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}

Float ABDLinearSubsystem::Impl::mass_norm()
{
    auto mass = abd().body_id_to_abd_mass.view();
    block_norm.resize(mass.size());
    {
        auto k = abd_linear_subsystem_mass_norm_kernel;
        int  n = (int)mass.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mass.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void ABDLinearSubsystem::do_init(InitInfo& info)
{
    m_impl.init();
}

void ABDLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void ABDLinearSubsystem::do_prelaunch_assembly(GlobalLinearSystem::PrelaunchInfo& info)
{
    m_impl.prelaunch_assembly(info);
}

void ABDLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void ABDLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void ABDLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

Float ABDLinearSubsystem::do_diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.diag_norm();
}

Float ABDLinearSubsystem::do_mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.mass_norm();
}

U64 ABDLinearSubsystem::get_uid() const noexcept
{
    return ABDLinearSubsystemUID;
}

void ABDLinearSubsystem::add_reporter(ABDLinearSubsystemReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter cannot be null");
    check_state(SimEngineState::BuildSystems, "add_reporter");
    m_impl.reporters.register_sim_system(*reporter);
}

void ABDLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void ABDLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}

ABDLinearSubsystem::AssembleInfo::AssembleInfo(Impl*  impl,
                                               IndexT index,
                                               bool   gradient_only,
                                               cuda_tool::SideStreams* side_streams) noexcept
    : m_impl(impl)
    , m_index(index)
    , m_gradient_only(gradient_only)
    , m_side_streams(side_streams)
{
}

cuda_tool::SideStreams* ABDLinearSubsystem::AssembleInfo::side_streams() const noexcept
{
    return m_side_streams;
}

cuda_tool::DoubletVectorView<Float, 12> ABDLinearSubsystem::AssembleInfo::gradients() const
{
    auto [offset, count] = m_impl->reporter_gradient_offsets_counts[m_index];
    return m_impl->reporter_gradients.view().subview(offset, count);
}

cuda_tool::TripletMatrixView<Float, 12, 12> ABDLinearSubsystem::AssembleInfo::hessians() const
{
    auto [offset, count] = m_impl->reporter_hessian_offsets_counts[m_index];
    return m_impl->reporter_hessians.view().subview(offset, count);
}

bool ABDLinearSubsystem::AssembleInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

void ABDLinearSubsystem::ReportExtentInfo::gradient_count(SizeT size)
{
    m_gradient_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::hessian_count(SizeT size)
{
    m_hessian_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::check(std::string_view name) const
{
    check_report_extent(m_gradient_only_checked, m_gradient_only, m_hessian_count, name);
}

AffineBodyDynamics::Impl& ABDLinearSubsystem::Impl::abd() const noexcept
{
    return affine_body_dynamics->m_impl;
}
}  // namespace uipc::backend::cuda
