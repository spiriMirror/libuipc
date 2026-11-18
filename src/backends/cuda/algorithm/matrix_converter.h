/*
 * NVIDIA contributions: compact radix-key and symmetric-to-general conversion
 * interfaces and state; the convert_upper interface, its device-side counts
 * and read-back state.
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
struct MatrixConverterIntPair
{
    int x;
    int y;
};

struct MatrixConverterRadixKeyConfig
{
    int      rows;
    int      cols;
    int      col_bits;
    int      end_bit;
    uint64_t col_mask;
    bool     compact;
};

// Valid indices occupy [0, extent), while extent itself encodes the -1
// sentinel.  Computing the width from extent (rather than extent - 1)
// reserves that extra code without evaluating extent + 1, which could
// overflow when extent == INT_MAX.
constexpr int matrix_converter_radix_code_bits(int extent)
{
    auto value = extent > 0 ? static_cast<uint32_t>(extent) : uint32_t{0};
    int  bits  = 1;
    while(value >>= 1)
        ++bits;
    return bits;
}

inline UIPC_GENERIC constexpr bool matrix_converter_radix_index_is_valid(int index, int extent)
{
    return index >= -1 && index < extent;
}

constexpr MatrixConverterRadixKeyConfig matrix_converter_radix_key_config(int rows, int cols)
{
    if(rows <= 0 || cols <= 0)
        return {rows, cols, 32, 64, 0xFFFFFFFFull, false};

    const int col_bits = matrix_converter_radix_code_bits(cols);
    const int end_bit  = col_bits + matrix_converter_radix_code_bits(rows);
    if(col_bits >= 64 || end_bit > 64)
        return {rows, cols, 32, 64, 0xFFFFFFFFull, false};

    return {rows, cols, col_bits, end_bit, (uint64_t{1} << col_bits) - uint64_t{1}, true};
}

inline UIPC_GENERIC constexpr uint64_t matrix_converter_pack_radix_key(
    int row, int col, const MatrixConverterRadixKeyConfig& config)
{
    if(!config.compact)
    {
        return (static_cast<uint64_t>(static_cast<uint32_t>(row)) << config.col_bits)
               | static_cast<uint64_t>(static_cast<uint32_t>(col));
    }

    const auto row_code = row == -1 ? static_cast<uint32_t>(config.rows) :
                                      static_cast<uint32_t>(row);
    const auto col_code = col == -1 ? static_cast<uint32_t>(config.cols) :
                                      static_cast<uint32_t>(col);
    return (static_cast<uint64_t>(row_code) << config.col_bits)
           | static_cast<uint64_t>(col_code);
}

inline UIPC_GENERIC constexpr MatrixConverterIntPair matrix_converter_unpack_radix_key(
    uint64_t key, const MatrixConverterRadixKeyConfig& config)
{
    const auto row_code = static_cast<uint32_t>(key >> config.col_bits);
    const auto col_code = static_cast<uint32_t>(key & config.col_mask);
    if(!config.compact)
        return {static_cast<int>(row_code), static_cast<int>(col_code)};

    return {row_code == static_cast<uint32_t>(config.rows) ? -1 : static_cast<int>(row_code),
            col_code == static_cast<uint32_t>(config.cols) ? -1 : static_cast<int>(col_code)};
}

// Key of every lower-triangle triplet in MatrixConverter::convert_upper: one
// row code past the -1 code, so it is unequal to and sorts after every key
// matrix_converter_pack_radix_key produces for a compact config.
inline UIPC_GENERIC constexpr uint64_t matrix_converter_radix_tail_key(const MatrixConverterRadixKeyConfig& config)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(config.rows)) + 1)
           << config.col_bits;
}

// Radix sort width that holds the tail key (and therefore every packed key).
constexpr int matrix_converter_radix_tail_end_bit(const MatrixConverterRadixKeyConfig& config)
{
    auto value = static_cast<uint64_t>(static_cast<uint32_t>(config.rows)) + 1;
    int  bits  = 0;
    while(value)
    {
        value >>= 1;
        ++bits;
    }
    return config.col_bits + bits;
}

constexpr bool operator==(const MatrixConverterIntPair& l, const MatrixConverterIntPair& r)
{
    return l.x == r.x && l.y == r.y;
}

// Device-side result sizes of MatrixConverter::convert_upper.
struct MatrixConverterUpperCounts
{
    int unique_count;  // blocks in the converted BCOO
    int upper_count;   // input triplets with row <= col
};

template <typename T, int N>
class MatrixConverter
{
    using BlockMatrix   = cuda_tool::DeviceTripletMatrix<T, N>::ValueT;
    using SegmentVector = cuda_tool::DeviceDoubletVector<T, N>::ValueT;

    Float m_reserve_ratio = 1.5;

    cuda_tool::DeviceBuffer<int> col_counts_per_row;
    cuda_tool::DeviceBuffer<int> unique_indices;
    cuda_tool::DeviceBuffer<int> unique_counts;
    cuda_tool::DeviceVar<int>    count;

    cuda_tool::DeviceBuffer<int> sort_index_input;
    cuda_tool::DeviceBuffer<int> sort_index;

    cuda_tool::DeviceBuffer<int> offsets;

    cuda_tool::DeviceBuffer<MatrixConverterIntPair> ij_pairs;
    cuda_tool::DeviceBuffer<MatrixConverterIntPair> unique_ij_pairs;

    cuda_tool::DeviceBuffer<uint64_t> ij_hash_input;
    cuda_tool::DeviceBuffer<uint64_t> ij_hash;

    cuda_tool::DeviceBuffer<BlockMatrix> blocks_sorted;
    cuda_tool::DeviceBuffer<BlockMatrix> diag_blocks;


    cuda_tool::DeviceBuffer<int>           indices_sorted;
    cuda_tool::DeviceBuffer<SegmentVector> segments_sorted;


    cuda_tool::DeviceBuffer<int> sorted_partition_input;
    cuda_tool::DeviceBuffer<int> sorted_partition_output;

    // convert_upper's counts: read on the device by its later kernels; k3
    // also stores them into the read-back buffer for the host
    cuda_tool::DeviceVar<MatrixConverterUpperCounts>      upper_counts;
    cuda_tool::ReadbackBuffer<MatrixConverterUpperCounts> upper_counts_readback;

  public:
    void  reserve_ratio(Float ratio) { m_reserve_ratio = ratio; }
    Float reserve_ratio() const { return m_reserve_ratio; }


    // Triplet -> BCOO
    void convert(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                 cuda_tool::DeviceBCOOMatrix<T, N>&          to);

    void _radix_sort_indices_and_blocks(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                        cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void _radix_sort_indices_and_blocks(cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void _make_unique_indices(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                              cuda_tool::DeviceBCOOMatrix<T, N>&          to);

    void _make_unique_block_warp_reduction(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                           cuda_tool::DeviceBCOOMatrix<T, N>& to);

    // BCOO -> BSR
    void convert(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                 cuda_tool::DeviceBSRMatrix<T, N>&        to);

    void _calculate_block_offsets(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                  cuda_tool::DeviceBSRMatrix<T, N>&        to);


    // Doublet -> BCOO
    void convert(const cuda_tool::DeviceDoubletVector<T, N>& from,
                 cuda_tool::DeviceBCOOVector<T, N>&          to);

    void _radix_sort_indices_and_segments(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                          cuda_tool::DeviceBCOOVector<T, N>& to);

    void _make_unique_indices(const cuda_tool::DeviceDoubletVector<T, N>& from,
                              cuda_tool::DeviceBCOOVector<T, N>&          to);

    void _make_unique_segment_warp_reduction(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                             cuda_tool::DeviceBCOOVector<T, N>& to);


    template <typename U>
    void loose_resize(cuda_tool::DeviceBuffer<U>& buf, size_t new_size)
    {
        if(buf.capacity() < new_size)
            buf.reserve_discard(new_size * m_reserve_ratio);
        buf.resize_discard(new_size);
    }

    void ge2sym(cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void ge2sym(cuda_tool::DeviceTripletMatrix<T, N>& to);

    // General Triplet -> upper-triangular BCOO (row <= col on the signed
    // indices). Bitwise the result of ge2sym(from) followed by
    // convert(from, to), without compacting or copying the input blocks:
    // lower triplets get the tail key and sort behind the upper ones, and the
    // segmented reduction reads each block through the sort permutation.
    // Requires rows > 0 and cols > 0 (a compact radix key).
    void convert_upper(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                       cuda_tool::DeviceBCOOMatrix<T, N>&          to);

    void sym2ge(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                cuda_tool::DeviceBCOOMatrix<T, N>&       to);
};
}  // namespace uipc::backend::cuda

#include "details/matrix_converter.inl"
