/*
 * NVIDIA contributions: one segment analysis per warp and the reduction into
 * an uncleared output.
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

#include <cub/util_ptx.cuh>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda_tool
{
namespace details::fast_segmental_reduce
{
    __host__ __device__ constexpr int b2i(bool b)
    {
        return b ? 1 : 0;
    }

    // Head-segmented warp reduction of the Count values of each lane, each
    // with exactly the steps of CUB's WarpReduce<T, 32>::HeadSegmentedReduce
    // (shuffle variant): at offsets 1, 2, 4, 8 and 16 a lane replaces its
    // value v by op(v, w), w the value of lane + offset, when that lane is in
    // its segment. No lane changes anything at an offset of at least the
    // warp's longest segment, so those steps are skipped (warp-uniformly).
    // One loop serves all values, so their shuffle chains interleave.
    template <int Count, typename T, typename ReduceOp>
    __device__ __forceinline__ void head_segmented_reduce(
        T* values, const WarpHeadSegments& segments, int lane_id, ReduceOp op)
    {
#pragma unroll
        for(int offset = 1; offset < 32; offset <<= 1)
        {
            if(offset >= segments.max_len)
                break;
            T other[Count];
#pragma unroll
            for(int k = 0; k < Count; ++k)
                other[k] = cub::ShuffleDown<32>(values[k], offset, segments.last_lane, 0xffffffffu);
            if(offset + lane_id <= segments.last_lane)
            {
#pragma unroll
                for(int k = 0; k < Count; ++k)
                    values[k] = op(values[k], other[k]);
            }
        }
    }

    // A segment continues across warps when its first lane is lane 0 and
    // continues the previous warp's last segment, or its last lane is lane 31
    // and the next warp's lane 0 continues it. The lanes past in_size carry
    // key -1 and no previous key, so they join the last segment, and when
    // they reach lane 31 that segment counts as continuing too.
    __device__ __forceinline__ bool continues_across_warps(
        int prev_i, int i, int next_i, int lane_id, const WarpHeadSegments& segments)
    {
        const int last_continues = __shfl_sync(0xffffffffu, b2i(next_i == i), 31);
        return (lane_id == 0 && prev_i == i) || (segments.last_lane == 31 && last_continues);
    }
}  // namespace details::fast_segmental_reduce

namespace
{
    // get-key functor over a segment-offset buffer (replaces the device lambda)
    struct fast_segmental_reduce_get_offset_key_op
    {
        CBufferView<int> offset;
        __device__ int   operator()(int i) const { return offset(i); }
    };

    // get-value functor over an input buffer (replaces the device lambda)
    template <typename T>
    struct fast_segmental_reduce_get_buffer_value_op
    {
        CBufferView<T> in;
        __device__ T   operator()(int i) const { return in(i); }
    };

    // FastSegmentalReduce::reduce (scalar values). Each warp reduces the
    // segments of its 32 inputs (as CUB's HeadSegmentedReduce: segments
    // start where the key changes and at lane 0); the head of a segment
    // stores its result, or adds it atomically when the segment continues
    // across warps.
    template <int BlockSize, int WarpSize, typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
    __global__ void fast_segmental_reduce_scalar_kernel(BufferView<T> out,
                                                        size_t        in_size,
                                                        GetKeyOp   get_key_op,
                                                        GetValueOp get_value_op,
                                                        ReduceOp   op)
    {
        using namespace details::fast_segmental_reduce;
        using ValueT = T;

        auto global_thread_id = blockDim.x * blockIdx.x + threadIdx.x;
        int  lane_id          = threadIdx.x & (WarpSize - 1);

        int    prev_i = -1;
        int    next_i = -1;
        int    i      = -1;
        bool   valid  = false;
        ValueT value;

        if(global_thread_id > 0 && global_thread_id < in_size)
        {
            prev_i = get_key_op(global_thread_id - 1);
        }

        if(global_thread_id < in_size - 1)
        {
            next_i = get_key_op(global_thread_id + 1);
        }

        if(global_thread_id < in_size)
        {
            i     = get_key_op(global_thread_id);
            value = get_value_op(global_thread_id);
            valid = true;
        }
        else
        {
            i     = -1;
            value = ValueT{0};
        }

        const int              is_head  = lane_id == 0 ? 1 : b2i(prev_i != i);
        const WarpHeadSegments segments = warp_head_segments(is_head, lane_id);
        const bool cross = continues_across_warps(prev_i, i, next_i, lane_id, segments);

        head_segmented_reduce<1>(&value, segments, lane_id, op);

        if(is_head && valid)
        {
            if(cross)
            {
                auto& out_value = out(i);
                atomic_add(&out_value, value);
            }
            else
            {
                out(i) = value;
            }
        }
    }

    // FastSegmentalReduce::reduce (Eigen::Matrix values), as the scalar
    // kernel with the M * N entries reduced in one loop
    template <int BlockSize, int WarpSize, typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
    __global__ void fast_segmental_reduce_matrix_kernel(BufferView<Eigen::Matrix<T, M, N>> out,
                                                        size_t     in_size,
                                                        GetKeyOp   get_key_op,
                                                        GetValueOp get_value_op,
                                                        ReduceOp   op)
    {
        using namespace details::fast_segmental_reduce;
        using Matrix = Eigen::Matrix<T, M, N>;

        auto global_thread_id = blockDim.x * blockIdx.x + threadIdx.x;
        int  lane_id          = threadIdx.x & (WarpSize - 1);

        int    prev_i = -1;
        int    next_i = -1;
        int    i      = -1;
        bool   valid  = false;
        Matrix value;

        if(global_thread_id > 0 && global_thread_id < in_size)
        {
            prev_i = get_key_op(global_thread_id - 1);
        }

        if(global_thread_id < in_size - 1)
        {
            next_i = get_key_op(global_thread_id + 1);
        }

        if(global_thread_id < in_size)
        {
            i     = get_key_op(global_thread_id);
            value = get_value_op(global_thread_id);
            valid = true;
        }
        else
        {
            i = -1;
            value.setZero();
        }

        const int              is_head  = lane_id == 0 ? 1 : b2i(prev_i != i);
        const WarpHeadSegments segments = warp_head_segments(is_head, lane_id);
        const bool cross = continues_across_warps(prev_i, i, next_i, lane_id, segments);

        head_segmented_reduce<M * N>(value.data(), segments, lane_id, op);

        if(is_head && valid)
        {
            if(cross)
            {
                auto& out_value = out(i);
                eigen::atomic_add(out_value, value);
            }
            else
            {
                out(i) = value;
            }
        }
    }
}  // namespace

template <int BlockSize, int WarpSize>
template <typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    size_t in_size, BufferView<T> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using ValueT = T;

    BufferLaunch(this->stream()).fill<ValueT>(out, ValueT{0});
    return reduce_uncleared(in_size, out, get_key_op, get_value_op, op);
}

template <int BlockSize, int WarpSize>
template <typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce_uncleared(
    size_t in_size, BufferView<T> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");
    static_assert(BlockSize % WarpSize == 0, "warps are aligned at multiples of WarpSize");

    auto          size      = in_size;
    constexpr int block_dim = BlockSize;

    int block_count = (size + block_dim - 1) / block_dim;
    if(block_count > 0)
        fast_segmental_reduce_scalar_kernel<BlockSize, WarpSize, T>
            <<<block_count, block_dim, 0, this->stream()>>>(out, size, get_key_op, get_value_op, op);

    return *this;
}


template <int BlockSize, int WarpSize>
template <typename T, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    CBufferView<int> offset, CBufferView<T> in, BufferView<T> out, ReduceOp op)
{
    return reduce(in.size(),
                  out,
                  fast_segmental_reduce_get_offset_key_op{offset},
                  fast_segmental_reduce_get_buffer_value_op<T>{in},
                  op);
}


template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    size_t in_size, BufferView<Eigen::Matrix<T, M, N>> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using Matrix = Eigen::Matrix<T, M, N>;

    BufferLaunch(this->stream()).fill<Matrix>(out, Matrix::Zero().eval());
    return reduce_uncleared(in_size, out, get_key_op, get_value_op, op);
}

template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce_uncleared(
    size_t in_size, BufferView<Eigen::Matrix<T, M, N>> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");
    static_assert(BlockSize % WarpSize == 0, "warps are aligned at multiples of WarpSize");

    auto          size      = in_size;
    constexpr int block_dim = BlockSize;

    int block_count = (size + block_dim - 1) / block_dim;
    if(block_count > 0)
        fast_segmental_reduce_matrix_kernel<BlockSize, WarpSize, T, M, N>
            <<<block_count, block_dim, 0, this->stream()>>>(out, size, get_key_op, get_value_op, op);

    return *this;
}

template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    CBufferView<int>                    offset,
    CBufferView<Eigen::Matrix<T, M, N>> in,
    BufferView<Eigen::Matrix<T, M, N>>  out,
    ReduceOp                            op)
{
    return reduce(in.size(),
                  out,
                  fast_segmental_reduce_get_offset_key_op{offset},
                  fast_segmental_reduce_get_buffer_value_op<Eigen::Matrix<T, M, N>>{in},
                  op);
}
}  // namespace uipc::backend::cuda_tool
