/*
 * NVIDIA contributions: compact radix-key ranges, sentinel handling and
 * symmetric-to-general sparse conversion changes; the upper-triangle
 * conversion without compaction (convert_upper), the reduction into an
 * uncleared output and device-side counts stored directly into read-back
 * buffers.
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

#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <cub/warp/warp_reduce.cuh>
#include <uipc/common/log.h>
#include <uipc/common/timer.h>
#include <algorithm/fast_segmental_reduce.h>

namespace uipc::backend::cuda
{
namespace
{
    // block value type of DeviceTripletMatrix<T, N> (plain T when N == 1)
    template <typename T, int N>
    using MatrixConverterBlockT = typename cuda_tool::DeviceTripletMatrix<T, N>::ValueT;

    // MatrixConverter::_radix_sort_indices_and_blocks(from, to) #1: hash ij
    __global__ void matrix_converter_radix_sort_indices_and_blocks_k1_kernel(
        cuda_tool::CBufferView<int>     row_indices,
        cuda_tool::CBufferView<int>     col_indices,
        cuda_tool::BufferView<uint64_t> ij_hash,
        cuda_tool::BufferView<int>      sort_index,
        MatrixConverterRadixKeyConfig   radix_key_config,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const int row = row_indices(i);
        const int col = col_indices(i);
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(row, radix_key_config.rows),
            "MatrixConverter row index is outside [-1, rows)");
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(col, radix_key_config.cols),
            "MatrixConverter column index is outside [-1, cols)");
        ij_hash(i) = matrix_converter_pack_radix_key(row, col, radix_key_config);
        sort_index(i) = i;
    }

    // MatrixConverter::_radix_sort_indices_and_blocks(from, to) #2: unpack ij hash
    __global__ void matrix_converter_radix_sort_indices_and_blocks_k2_kernel(
        cuda_tool::BufferView<uint64_t>               ij_hash,
        cuda_tool::BufferView<MatrixConverterIntPair> ij_pairs,
        MatrixConverterRadixKeyConfig                 radix_key_config,
        int                                           n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        ij_pairs(i) = matrix_converter_unpack_radix_key(ij_hash(i), radix_key_config);
    }

    // MatrixConverter::_radix_sort_indices_and_blocks(from, to) #3: sort the block values
    template <typename T, int N>
    __global__ void matrix_converter_radix_sort_indices_and_blocks_k3_kernel(
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> src_blocks,
        cuda_tool::CBufferView<int>                         sort_index,
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  dst_blocks,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dst_blocks(i) = src_blocks(sort_index(i));
    }

    // MatrixConverter::_radix_sort_indices_and_blocks(to) #1: hash ij
    __global__ void matrix_converter_radix_sort_indices_and_blocks_in_place_k1_kernel(
        cuda_tool::CBufferView<int>     row_indices,
        cuda_tool::CBufferView<int>     col_indices,
        cuda_tool::BufferView<uint64_t> ij_hash,
        cuda_tool::BufferView<int>      sort_index,
        MatrixConverterRadixKeyConfig   radix_key_config,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const int row = row_indices(i);
        const int col = col_indices(i);
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(row, radix_key_config.rows),
            "MatrixConverter row index is outside [-1, rows)");
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(col, radix_key_config.cols),
            "MatrixConverter column index is outside [-1, cols)");
        ij_hash(i) = matrix_converter_pack_radix_key(row, col, radix_key_config);
        sort_index(i) = i;
    }

    // MatrixConverter::_radix_sort_indices_and_blocks(to) #2: unpack ij hash
    __global__ void matrix_converter_radix_sort_indices_and_blocks_in_place_k2_kernel(
        cuda_tool::BufferView<uint64_t>               ij_hash,
        cuda_tool::BufferView<MatrixConverterIntPair> ij_pairs,
        MatrixConverterRadixKeyConfig                 radix_key_config,
        int                                           n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        ij_pairs(i) = matrix_converter_unpack_radix_key(ij_hash(i), radix_key_config);
    }

    // MatrixConverter::_radix_sort_indices_and_blocks(to) #3: sort the block values
    template <typename T, int N>
    __global__ void matrix_converter_radix_sort_indices_and_blocks_in_place_k3_kernel(
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> src_blocks,
        cuda_tool::CBufferView<int>                         sort_index,
        cuda_tool::CBufferView<MatrixConverterIntPair>      ij_pairs,
        cuda_tool::BufferView<int>                          dst_row,
        cuda_tool::BufferView<int>                          dst_col,
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  dst_blocks,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dst_blocks(i) = src_blocks(sort_index(i));
        dst_row(i)    = ij_pairs(i).x;
        dst_col(i)    = ij_pairs(i).y;
    }

    // MatrixConverter::_make_unique_indices(triplet -> bcoo) #1
    __global__ void matrix_converter_make_unique_indices_k1_kernel(
        cuda_tool::BufferView<MatrixConverterIntPair> unique_ij_pairs,
        cuda_tool::BufferView<int>                    row_indices,
        cuda_tool::BufferView<int>                    col_indices,
        int                                           n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        row_indices(i) = unique_ij_pairs(i).x;
        col_indices(i) = unique_ij_pairs(i).y;
    }

    // MatrixConverter::_make_unique_block_warp_reduction #1: mark segment tails
    __global__ void matrix_converter_make_unique_block_warp_reduction_k1_kernel(
        cuda_tool::BufferView<int> sorted_partition,
        cuda_tool::BufferView<int> unique_counts,
        cuda_tool::BufferView<int> offsets,
        int                        n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto offset = offsets(i);
        auto count  = unique_counts(i);

        sorted_partition(offset + count - 1) = 1;
    }

    // MatrixConverter::_calculate_block_offsets #1: scatter run lengths per row
    __global__ void matrix_converter_calculate_block_offsets_k1_kernel(
        cuda_tool::CBufferView<int> unique_indices,
        cuda_tool::BufferView<int>  counts,
        cuda_tool::BufferView<int>  col_counts_per_row,
        int                         n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto row                = unique_indices(i);
        col_counts_per_row(row) = counts(i);
    }

    // MatrixConverter::_make_unique_indices(doublet -> bcoo vector) #1
    __global__ void matrix_converter_make_unique_vector_indices_k1_kernel(
        cuda_tool::BufferView<int> unique_indices, cuda_tool::BufferView<int> dst_indices, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dst_indices(i) = unique_indices(i);
    }

    // MatrixConverter::_make_unique_segment_warp_reduction #1: mark segment tails
    __global__ void matrix_converter_make_unique_segment_warp_reduction_k1_kernel(
        cuda_tool::BufferView<int> sorted_partition,
        cuda_tool::BufferView<int> unique_counts,
        cuda_tool::BufferView<int> offsets,
        int                        n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto offset = offsets(i);
        auto count  = unique_counts(i);

        sorted_partition(offset + count - 1) = 1;
    }

    // MatrixConverter::ge2sym(bcoo) #1: find the upper triangular part (i <= j)
    template <typename T, int N>
    __global__ void matrix_converter_ge2sym_bcoo_k1_kernel(
        cuda_tool::CBufferView<int>                         row_indices,
        cuda_tool::CBufferView<int>                         col_indices,
        cuda_tool::BufferView<MatrixConverterIntPair>       ij_pairs,
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> blocks,
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  block_temp,
        cuda_tool::BufferView<int>                          counts,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        counts(i)     = row_indices(i) <= col_indices(i) ? 1 : 0;
        ij_pairs(i).x = row_indices(i);
        ij_pairs(i).y = col_indices(i);
        block_temp(i) = blocks(i);
    }

    // MatrixConverter::ge2sym(bcoo) #2: compact the upper triangular part
    template <typename T, int N>
    __global__ void matrix_converter_ge2sym_bcoo_k2_kernel(
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  dst_blocks,
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> src_blocks,
        cuda_tool::CBufferView<MatrixConverterIntPair>      ij_pairs,
        cuda_tool::BufferView<int>                          row_indices,
        cuda_tool::BufferView<int>                          col_indices,
        cuda_tool::CBufferView<int>                         counts,
        cuda_tool::CBufferView<int>                         offsets,
        cuda_tool::Dense<int>                               total_count,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto count  = counts(i);
        auto offset = offsets(i);

        if(count != 0)
        {
            dst_blocks(offset)  = src_blocks(i);
            auto ij             = ij_pairs(i);
            row_indices(offset) = ij.x;
            col_indices(offset) = ij.y;
        }

        if(i == offsets.total_size() - 1)
        {
            total_count = offsets(i) + counts(i);
        }
    }

    // MatrixConverter::ge2sym(triplet) #1: find the upper triangular part (i <= j)
    template <typename T, int N>
    __global__ void matrix_converter_ge2sym_triplet_k1_kernel(
        cuda_tool::CBufferView<int>                         row_indices,
        cuda_tool::CBufferView<int>                         col_indices,
        cuda_tool::BufferView<MatrixConverterIntPair>       ij_pairs,
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> blocks,
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  block_temp,
        cuda_tool::BufferView<int>                          counts,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        counts(i)     = row_indices(i) <= col_indices(i) ? 1 : 0;
        ij_pairs(i).x = row_indices(i);
        ij_pairs(i).y = col_indices(i);
        block_temp(i) = blocks(i);
    }

    // MatrixConverter::ge2sym(triplet) #2: compact the upper triangular part
    template <typename T, int N>
    __global__ void matrix_converter_ge2sym_triplet_k2_kernel(
        cuda_tool::BufferView<MatrixConverterBlockT<T, N>>  dst_blocks,
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> src_blocks,
        cuda_tool::CBufferView<MatrixConverterIntPair>      ij_pairs,
        cuda_tool::BufferView<int>                          row_indices,
        cuda_tool::BufferView<int>                          col_indices,
        cuda_tool::CBufferView<int>                         counts,
        cuda_tool::CBufferView<int>                         offsets,
        cuda_tool::Dense<int>                               total_count,
        int                                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto count  = counts(i);
        auto offset = offsets(i);

        if(count != 0)
        {
            dst_blocks(offset)  = src_blocks(i);
            auto ij             = ij_pairs(i);
            row_indices(offset) = ij.x;
            col_indices(offset) = ij.y;
        }

        if(i == offsets.total_size() - 1)
        {
            total_count = offsets(i) + counts(i);
        }
    }

    // MatrixConverter::convert_upper #1: hash ij, lower triplets get the tail key
    __global__ void matrix_converter_convert_upper_k1_kernel(
        cuda_tool::CBufferView<int>     row_indices,
        cuda_tool::CBufferView<int>     col_indices,
        cuda_tool::BufferView<uint64_t> ij_hash,
        cuda_tool::BufferView<int>      sort_index,
        MatrixConverterRadixKeyConfig   radix_key_config,
        uint64_t                        tail_key,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const int row = row_indices(i);
        const int col = col_indices(i);
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(row, radix_key_config.rows),
            "MatrixConverter row index is outside [-1, rows)");
        UIPC_KERNEL_ASSERT(
            matrix_converter_radix_index_is_valid(col, radix_key_config.cols),
            "MatrixConverter column index is outside [-1, cols)");
        // same signed test as ge2sym: (-1, j) is kept, (i >= 0, -1) is dropped
        ij_hash(i)    = row <= col ?
                            matrix_converter_pack_radix_key(row, col, radix_key_config) :
                            tail_key;
        sort_index(i) = i;
    }

    // MatrixConverter::convert_upper #2: flag segment heads, find the upper prefix
    __global__ void matrix_converter_convert_upper_k2_kernel(
        cuda_tool::CBufferView<uint64_t>             sorted_ij_hash,
        cuda_tool::BufferView<int>                   heads,
        cuda_tool::Dense<MatrixConverterUpperCounts> counts,
        uint64_t                                     tail_key,
        int                                          n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const uint64_t key  = sorted_ij_hash(i);
        const bool     head = i > 0 && key != sorted_ij_hash(i - 1);
        heads(i)            = head ? 1 : 0;

        // exactly one thread ends the upper prefix: the first tail key, or the
        // last triplet when there is none
        const bool tail = key == tail_key;
        if(tail ? (i == 0 || head) : i == n - 1)
            counts->upper_count = tail ? i : n;
    }

    // MatrixConverter::convert_upper #3: unique count from the segment index;
    // the counts also go to the host's read-back slot
    __global__ void matrix_converter_convert_upper_k3_kernel(
        cuda_tool::CBufferView<int>                  segments,
        cuda_tool::Dense<MatrixConverterUpperCounts> counts,
        cuda_tool::Dense<MatrixConverterUpperCounts> readback)
    {
        const int upper      = counts->upper_count;
        counts->unique_count = upper > 0 ? segments(upper - 1) + 1 : 0;
        *readback            = *counts;
    }

    // MatrixConverter::convert_upper #4: unpack the ij of every segment head,
    // and clear the block of every segment the segmented reduction adds to
    // atomically instead of storing: one that spans two of its warps
    // (warp_size consecutive triplets, aligned), and the last one when the
    // last warp is partial (the reduction's lanes past n join the last
    // segment and flag it as continuing).
    template <typename BlockT>
    __global__ void matrix_converter_convert_upper_k4_kernel(
        cuda_tool::CBufferView<uint64_t> sorted_ij_hash,
        cuda_tool::CBufferView<int>      segments,
        cuda_tool::BufferView<int>       row_indices,
        cuda_tool::BufferView<int>       col_indices,
        cuda_tool::BufferView<BlockT>    values,
        MatrixConverterRadixKeyConfig    radix_key_config,
        int                              warp_size,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const int segment = segments(i);
        if(i > 0 && segments(i - 1) == segment)
            return;
        const auto ij = matrix_converter_unpack_radix_key(sorted_ij_hash(i), radix_key_config);
        row_indices(segment) = ij.x;
        col_indices(segment) = ij.y;
        const int next_warp  = (i / warp_size + 1) * warp_size;
        const bool spans_warps = next_warp < n && segments(next_warp) == segment;
        const bool partial_end = n % warp_size != 0 && segments(n - 1) == segment;
        if(spans_warps || partial_end)
        {
            if constexpr(std::is_arithmetic_v<BlockT>)
                values(segment) = BlockT{0};
            else
                values(segment) = BlockT::Zero();
        }
    }

    // MatrixConverter::convert_upper: FastSegmentalReduce value op that reads
    // the input block at each sorted position
    template <typename T, int N>
    struct matrix_converter_sorted_block_op
    {
        cuda_tool::CBufferView<MatrixConverterBlockT<T, N>> blocks;
        cuda_tool::CBufferView<int>                         sort_index;
        __device__ MatrixConverterBlockT<T, N> operator()(int i) const
        {
            return blocks(sort_index(i));
        }
    };

    // MatrixConverter::sym2ge #1: setup select flag
    __global__ void matrix_converter_sym2ge_k1_kernel(cuda_tool::BufferView<int> flags,
                                                      cuda_tool::CBufferView<int> row_indices,
                                                      cuda_tool::CBufferView<int> col_indices,
                                                      cuda_tool::BufferView<int> partition_index,
                                                      int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        flags(i)           = (row_indices(i) == col_indices(i)) ? 1 : 0;
        partition_index(i) = i;
    }

    // MatrixConverter::sym2ge #2: copy blocks and ij as [ Diag | Upper | Lower ]
    template <typename T, int N>
    __global__ void matrix_converter_sym2ge_k2_kernel(cuda_tool::BCOOMatrixView<T, N> to,
                                                      cuda_tool::CBCOOMatrixView<T, N> from,
                                                      cuda_tool::CBufferView<int> partition_index,
                                                      int diag_count,
                                                      int sym_size,
                                                      int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto index = partition_index(i);
        auto f     = from(index);
        // diag + upper
        to(i).write(f.row_index, f.col_index, f.value);
        if(i >= diag_count)
        {
            // lower
            to(i + sym_size - diag_count)
                .write(f.col_index, f.row_index, f.value.transpose());
        }
    }
}  // namespace

template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                    cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    to.reshape(from.rows(), from.cols());
    to.resize_triplets_discard(from.triplet_count());


    if(to.triplet_count() == 0)
        return;

    _radix_sort_indices_and_blocks(from, to);

    _make_unique_indices(from, to);
    _make_unique_block_warp_reduction(from, to);
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_blocks(
    const cuda_tool::DeviceTripletMatrix<T, N>& from, cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto src_row_indices = from.row_indices();
    auto src_col_indices = from.col_indices();
    auto src_blocks      = from.values();

    loose_resize(ij_hash_input, src_row_indices.size());
    loose_resize(sort_index_input, src_row_indices.size());

    loose_resize(ij_hash, src_row_indices.size());
    loose_resize(sort_index, src_row_indices.size());
    loose_resize(ij_pairs, src_row_indices.size());

    const auto radix_key_config =
        matrix_converter_radix_key_config(from.rows(), from.cols());

    // hash ij
    int n_hash_ij = (int)src_row_indices.size();
    if(n_hash_ij > 0)
        matrix_converter_radix_sort_indices_and_blocks_k1_kernel<<<(n_hash_ij + 256 - 1) / 256, 256, 0, nullptr>>>(
            src_row_indices,
            src_col_indices,
            ij_hash_input.view(),
            sort_index_input.view(),
            radix_key_config,
            n_hash_ij);

    DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                ij_hash.data(),
                                sort_index_input.data(),
                                sort_index.data(),
                                ij_hash.size(),
                                0,
                                radix_key_config.end_bit);

    // set ij_hash back to row_indices and col_indices

    auto dst_row_indices = to.row_indices();
    auto dst_col_indices = to.col_indices();

    int n_unpack_ij = (int)dst_row_indices.size();
    if(n_unpack_ij > 0)
        matrix_converter_radix_sort_indices_and_blocks_k2_kernel<<<(n_unpack_ij + 256 - 1) / 256, 256, 0, nullptr>>>(
            ij_hash.view(), ij_pairs.view(), radix_key_config, n_unpack_ij);

    // sort the block values

    {
        loose_resize(blocks_sorted, from.values().size());
        int n_sort_blocks = (int)src_blocks.size();
        if(n_sort_blocks > 0)
            matrix_converter_radix_sort_indices_and_blocks_k3_kernel<T, N>
                <<<(n_sort_blocks + 256 - 1) / 256, 256, 0, nullptr>>>(
                    src_blocks, sort_index.cview(), blocks_sorted.view(), n_sort_blocks);
    }
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_blocks(cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto src_row_indices = to.row_indices();
    auto src_col_indices = to.col_indices();
    auto src_blocks      = to.values();

    loose_resize(ij_hash_input, src_row_indices.size());
    loose_resize(sort_index_input, src_row_indices.size());

    loose_resize(ij_hash, src_row_indices.size());
    loose_resize(sort_index, src_row_indices.size());
    loose_resize(ij_pairs, src_row_indices.size());

    const auto radix_key_config =
        matrix_converter_radix_key_config(to.rows(), to.cols());

    // hash ij
    int n_hash_ij = (int)src_row_indices.size();
    if(n_hash_ij > 0)
        matrix_converter_radix_sort_indices_and_blocks_in_place_k1_kernel<<<(n_hash_ij + 256 - 1) / 256, 256, 0, nullptr>>>(
            src_row_indices.cview(),
            src_col_indices.cview(),
            ij_hash_input.view(),
            sort_index_input.view(),
            radix_key_config,
            n_hash_ij);

    DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                ij_hash.data(),
                                sort_index_input.data(),
                                sort_index.data(),
                                ij_hash.size(),
                                0,
                                radix_key_config.end_bit);

    // set ij_hash back to row_indices and col_indices

    auto dst_row_indices = to.row_indices();
    auto dst_col_indices = to.col_indices();

    int n_unpack_ij = (int)dst_row_indices.size();
    if(n_unpack_ij > 0)
        matrix_converter_radix_sort_indices_and_blocks_in_place_k2_kernel<<<(n_unpack_ij + 256 - 1) / 256, 256, 0, nullptr>>>(
            ij_hash.view(), ij_pairs.view(), radix_key_config, n_unpack_ij);

    // sort the block values

    {
        loose_resize(blocks_sorted, to.values().size());
        int n_sort_blocks = (int)src_blocks.size();
        if(n_sort_blocks > 0)
            matrix_converter_radix_sort_indices_and_blocks_in_place_k3_kernel<T, N>
                <<<(n_sort_blocks + 256 - 1) / 256, 256, 0, nullptr>>>(
                    src_blocks.cview(),
                    sort_index.cview(),
                    ij_pairs.cview(),
                    to.row_indices(),
                    to.col_indices(),
                    blocks_sorted.view(),
                    n_sort_blocks);

        to.values().copy_from(blocks_sorted);
    }
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_indices(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                                 cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto row_indices = to.row_indices();
    auto col_indices = to.col_indices();

    loose_resize(unique_ij_pairs, ij_pairs.size());
    loose_resize(unique_counts, ij_pairs.size());


    DeviceRunLengthEncode().Encode(ij_pairs.data(),
                                   unique_ij_pairs.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   ij_pairs.size());

    int h_count = count;

    unique_ij_pairs.resize_discard(h_count);
    unique_counts.resize_discard(h_count);

    loose_resize(offsets, unique_counts.size());

    DeviceScan().ExclusiveSum(
        unique_counts.data(), offsets.data(), unique_counts.size());


    int n_unique = (int)unique_counts.size();
    if(n_unique > 0)
        matrix_converter_make_unique_indices_k1_kernel<<<(n_unique + 256 - 1) / 256, 256, 0, nullptr>>>(
            unique_ij_pairs.view(), row_indices, col_indices, n_unique);

    to.resize_triplets_discard(h_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_block_warp_reduction(
    const cuda_tool::DeviceTripletMatrix<T, N>& from, cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    loose_resize(sorted_partition_input, ij_pairs.size());
    loose_resize(sorted_partition_output, ij_pairs.size());


    BufferLaunch().fill<int>(sorted_partition_input, 0);

    int n_mark = (int)unique_counts.size();
    if(n_mark > 0)
    {
        auto k = matrix_converter_make_unique_block_warp_reduction_k1_kernel;
        k<<<cuda_tool::best_grid_dim(n_mark, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            sorted_partition_input.view(), unique_counts.view(), offsets.view(), n_mark);
    }

    // scatter
    DeviceScan().ExclusiveSum(sorted_partition_input.data(),
                              sorted_partition_output.data(),
                              sorted_partition_input.size());

    auto blocks = to.values();

    FastSegmentalReduce<>().reduce(std::as_const(sorted_partition_output).view(),
                                   std::as_const(blocks_sorted).view(),
                                   blocks);
}

template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                    cuda_tool::DeviceBSRMatrix<T, N>& to)
{
    // calculate the row offsets
    _calculate_block_offsets(from, to);

    to.resize(from.non_zeros());

    auto vals        = to.values();
    auto col_indices = to.col_indices();

    vals.copy_from(from.values());  // BCOO and BSR have the same block values
    col_indices.copy_from(from.col_indices());  // BCOO and BSR have the same block col indices
}

template <typename T, int N>
void MatrixConverter<T, N>::_calculate_block_offsets(
    const cuda_tool::DeviceBCOOMatrix<T, N>& from, cuda_tool::DeviceBSRMatrix<T, N>& to)
{
    //Timer timer{__FUNCTION__};

    using namespace cuda_tool;
    to.reshape(from.rows(), from.cols());


    auto dst_row_offsets = to.row_offsets();

    loose_resize(col_counts_per_row, dst_row_offsets.size());
    col_counts_per_row.fill(0);

    loose_resize(unique_indices, from.non_zeros());
    loose_resize(unique_counts, from.non_zeros());


    // run length encode the row
    DeviceRunLengthEncode().Encode(from.row_indices().data(),
                                   unique_indices.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   from.non_zeros());
    int h_count = count;

    unique_indices.resize_discard(h_count);
    unique_counts.resize_discard(h_count);

    int n_scatter = (int)unique_counts.size();
    if(n_scatter > 0)
        matrix_converter_calculate_block_offsets_k1_kernel<<<(n_scatter + 256 - 1) / 256, 256, 0, nullptr>>>(
            unique_indices.cview(), unique_counts.view(), col_counts_per_row.view(), n_scatter);

    // calculate the offsets
    DeviceScan().ExclusiveSum(col_counts_per_row.data(),
                              dst_row_offsets.data(),
                              col_counts_per_row.size());
}

//using T         = Float;
//constexpr int N = 3;

template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                    cuda_tool::DeviceBCOOVector<T, N>& to)
{
    to.reshape(from.count());
    to.resize_doublets_discard(from.doublet_count());

    if(to.doublet_count() == 0)
        return;

    _radix_sort_indices_and_segments(from, to);
    _make_unique_indices(from, to);
    _make_unique_segment_warp_reduction(from, to);
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_segments(
    const cuda_tool::DeviceDoubletVector<T, N>& from, cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    auto src_indices  = from.indices();
    auto src_segments = from.values();

    loose_resize(indices_sorted, src_indices.size());
    loose_resize(segments_sorted, src_segments.size());

    DeviceRadixSort().SortPairs(src_indices.data(),
                                indices_sorted.data(),
                                src_segments.data(),
                                segments_sorted.data(),
                                src_indices.size());
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_indices(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                                 cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    auto dst_indices  = to.indices();
    auto dst_segments = to.values();
    loose_resize(unique_indices, indices_sorted.size());
    loose_resize(unique_counts, indices_sorted.size());

    DeviceRunLengthEncode().Encode(indices_sorted.data(),
                                   unique_indices.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   indices_sorted.size());

    int h_count = count;

    unique_indices.resize_discard(h_count);
    unique_counts.resize_discard(h_count);

    loose_resize(offsets, unique_counts.size());

    DeviceScan().ExclusiveSum(
        unique_counts.data(), offsets.data(), unique_counts.size());

    int n_unique = (int)unique_counts.size();
    if(n_unique > 0)
        matrix_converter_make_unique_vector_indices_k1_kernel<<<(n_unique + 256 - 1) / 256, 256, 0, nullptr>>>(
            unique_indices.view(), dst_indices, n_unique);

    to.resize_doublets_discard(h_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_segment_warp_reduction(
    const cuda_tool::DeviceDoubletVector<T, N>& from, cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    loose_resize(sorted_partition_input, indices_sorted.size());
    loose_resize(sorted_partition_output, indices_sorted.size());

    BufferLaunch().fill<int>(sorted_partition_input, 0);

    int n_mark = (int)unique_counts.size();
    if(n_mark > 0)
    {
        auto k = matrix_converter_make_unique_segment_warp_reduction_k1_kernel;
        k<<<cuda_tool::best_grid_dim(n_mark, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            sorted_partition_input.view(), unique_counts.view(), offsets.view(), n_mark);
    }

    // scatter
    DeviceScan().ExclusiveSum(sorted_partition_input.data(),
                              sorted_partition_output.data(),
                              sorted_partition_input.size());

    auto segments = to.values();

    FastSegmentalReduce<64, 32>().reduce(std::as_const(sorted_partition_output).view(),
                                         std::as_const(segments_sorted).view(),
                                         segments);
}

template <typename T, int N>
void MatrixConverter<T, N>::ge2sym(cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    // alias to reuse the memory
    auto& counts     = unique_counts;
    auto& block_temp = blocks_sorted;

    loose_resize(counts, to.non_zeros());
    loose_resize(offsets, to.non_zeros());
    loose_resize(ij_pairs, to.non_zeros());
    loose_resize(block_temp, to.values().size());

    // 0. find the upper triangular part (where i <= j)
    int n_upper = to.non_zeros();
    if(n_upper > 0)
    {
        auto k = matrix_converter_ge2sym_bcoo_k1_kernel<T, N>;
        k<<<cuda_tool::best_grid_dim(n_upper, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            to.row_indices().cview(),
            to.col_indices().cview(),
            ij_pairs.view(),
            to.values().cview(),
            block_temp.view(),
            counts.view(),
            n_upper);
    }

    // exclusive sum
    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());

    // set the values
    auto dst_block = to.values();

    int n_compact = (int)dst_block.size();
    if(n_compact > 0)
    {
        auto k = matrix_converter_ge2sym_bcoo_k2_kernel<T, N>;
        k<<<cuda_tool::best_grid_dim(n_compact, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dst_block,
            block_temp.cview(),
            ij_pairs.cview(),
            to.row_indices(),
            to.col_indices(),
            counts.cview(),
            offsets.cview(),
            count.viewer(),
            n_compact);
    }

    int h_total_count = count;

    to.resize_triplets_discard(h_total_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::ge2sym(cuda_tool::DeviceTripletMatrix<T, N>& to)
{
    using namespace cuda_tool;

    // alias to reuse the memory
    auto& counts     = unique_counts;
    auto& block_temp = blocks_sorted;

    loose_resize(counts, to.triplet_count());
    loose_resize(offsets, to.triplet_count());
    loose_resize(ij_pairs, to.triplet_count());
    loose_resize(block_temp, to.values().size());

    // 0. find the upper triangular part (where i <= j)
    int n_upper = (int)to.triplet_count();
    if(n_upper > 0)
    {
        auto k = matrix_converter_ge2sym_triplet_k1_kernel<T, N>;
        k<<<cuda_tool::best_grid_dim(n_upper, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            to.row_indices().cview(),
            to.col_indices().cview(),
            ij_pairs.view(),
            to.values().cview(),
            block_temp.view(),
            counts.view(),
            n_upper);
    }

    // exclusive sum
    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());

    // set the values
    auto dst_block = to.values();

    int n_compact = (int)dst_block.size();
    if(n_compact > 0)
    {
        auto k = matrix_converter_ge2sym_triplet_k2_kernel<T, N>;
        k<<<cuda_tool::best_grid_dim(n_compact, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dst_block,
            block_temp.cview(),
            ij_pairs.cview(),
            to.row_indices(),
            to.col_indices(),
            counts.cview(),
            offsets.cview(),
            count.viewer(),
            n_compact);
    }

    int h_total_count = count;

    to.resize_triplets_discard(h_total_count);
}


template <typename T, int N>
void MatrixConverter<T, N>::convert_upper(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                          cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    to.reshape(from.rows(), from.cols());

    const int n = (int)from.triplet_count();
    if(n == 0)
    {
        to.resize_triplets_discard(0);
        upper_counts = MatrixConverterUpperCounts{0, 0};
        return;
    }

    const auto radix_key_config =
        matrix_converter_radix_key_config(from.rows(), from.cols());
    UIPC_ASSERT(radix_key_config.compact,
                "MatrixConverter::convert_upper requires rows > 0 and cols > 0, got {}x{}",
                from.rows(),
                from.cols());
    const auto tail_key = matrix_converter_radix_tail_key(radix_key_config);

    loose_resize(ij_hash_input, n);
    loose_resize(sort_index_input, n);
    loose_resize(ij_hash, n);
    loose_resize(sort_index, n);

    // aliases to reuse the memory: segment heads and segment indices
    auto& heads    = sorted_partition_input;
    auto& segments = sorted_partition_output;
    loose_resize(heads, n);
    loose_resize(segments, n);

    matrix_converter_convert_upper_k1_kernel<<<(n + 256 - 1) / 256, 256, 0, nullptr>>>(
        from.row_indices(),
        from.col_indices(),
        ij_hash_input.view(),
        sort_index_input.view(),
        radix_key_config,
        tail_key,
        n);

    // stable: equal keys keep the input order, which is ge2sym's compaction order
    DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                ij_hash.data(),
                                sort_index_input.data(),
                                sort_index.data(),
                                n,
                                0,
                                matrix_converter_radix_tail_end_bit(radix_key_config));

    matrix_converter_convert_upper_k2_kernel<<<(n + 256 - 1) / 256, 256, 0, nullptr>>>(
        ij_hash.cview(), heads.view(), upper_counts.viewer(), tail_key, n);

    // segment index of every sorted triplet (the tail forms the last segment)
    DeviceScan().InclusiveSum(heads.data(), segments.data(), n);

    upper_counts_readback.resize_discard(1);
    matrix_converter_convert_upper_k3_kernel<<<1, 1, 0, nullptr>>>(
        segments.cview(),
        upper_counts.viewer(),
        cuda_tool::Dense<MatrixConverterUpperCounts>{upper_counts_readback.device_data()});
    upper_counts_readback.enqueue_copy(nullptr);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    const MatrixConverterUpperCounts h_counts = upper_counts_readback[0];

    to.resize_triplets_discard(h_counts.unique_count);

    const int n_upper = h_counts.upper_count;
    if(n_upper == 0)
        return;

    using Reduce = FastSegmentalReduce<>;
    matrix_converter_convert_upper_k4_kernel<<<(n_upper + 256 - 1) / 256, 256, 0, nullptr>>>(
        ij_hash.cview(),
        segments.cview(),
        to.row_indices(),
        to.col_indices(),
        to.values(),
        radix_key_config,
        Reduce::warp_size,
        n_upper);

    // k4 cleared the blocks the reduction accumulates into
    Reduce().reduce_uncleared(
        n_upper,
        to.values(),
        fast_segmental_reduce_get_offset_key_op{segments.cview()},
        matrix_converter_sorted_block_op<T, N>{from.values(), sort_index.cview()});
}

template <typename T, int N>
void MatrixConverter<T, N>::sym2ge(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                   cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto sym_size = from.non_zeros();

    // alias to reuse the memory
    auto& flags                 = offsets;
    auto& partitioned           = blocks_sorted;
    auto& partition_index_input = sort_index_input;
    auto& partition_index       = sort_index;
    auto& selected_count        = count;


    loose_resize(flags, sym_size);
    loose_resize(partitioned, sym_size);
    loose_resize(partition_index_input, sym_size);
    loose_resize(partition_index, sym_size);

    if(sym_size == 0)
    {
        to.resize_discard(from.rows(), from.cols(), 0);
        return;
    }

    // setup select flag
    if(sym_size > 0)
    {
        auto k = matrix_converter_sym2ge_k1_kernel;
        k<<<cuda_tool::best_grid_dim(sym_size, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            flags.view(),
            from.row_indices(),
            from.col_indices(),
            partition_index_input.view(),
            sym_size);
    }


    cuda_tool::DevicePartition().Flagged(partition_index_input.data(),
                                         flags.data(),
                                         partition_index.data(),
                                         selected_count.data(),
                                         sym_size);


    const int diag_count = selected_count;
    const size_t general_bcoo_size = size_t(sym_size) + size_t(sym_size - diag_count);

    to.resize_discard(from.rows(), from.cols(), general_bcoo_size);

    // copy blocks and ij
    // in this sequence:
    // [ Diag | Upper | Lower ]
    //
    if(sym_size > 0)
    {
        auto k = matrix_converter_sym2ge_k2_kernel<T, N>;
        k<<<cuda_tool::best_grid_dim(sym_size, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            to.view(), from.cview(), partition_index.cview(), diag_count, sym_size, sym_size);
    }

    _radix_sort_indices_and_blocks(to);
}
}  // namespace uipc::backend::cuda
