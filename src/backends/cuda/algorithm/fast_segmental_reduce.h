/*
 * NVIDIA contributions: interfaces of the per-warp segment analysis and the
 * reduction into an uncleared output.
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
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Core>
#include <cub/util_type.cuh>
#include <cuda/std/functional>
namespace uipc::backend::cuda_tool
{
// Segments of a warp as CUB's WarpReduce::HeadSegmentedReduce finds them:
// segments start at the lanes whose head flag is set (lane 0 always starts
// one).
struct WarpHeadSegments
{
    int last_lane;  // last lane of this lane's segment
    int max_len;    // longest segment of the warp
};

// Maximum of v over the 32 lanes of a full warp. __reduce_max_sync needs
// sm_80; the targets below it (the wheels build sm_75) take a butterfly of
// shuffles, which gives the same value. Shuffle = true takes the shuffles on
// every target, so a test can compare both forms on one GPU.
template <bool Shuffle = false>
__device__ __forceinline__ unsigned warp_max(unsigned v)
{
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ >= 800
    if constexpr(!Shuffle)
        return __reduce_max_sync(0xffffffffu, v);
    else
#endif
    {
        for(int offset = 16; offset > 0; offset /= 2)
        {
            const unsigned other = __shfl_xor_sync(0xffffffffu, v, offset);
            v                    = other > v ? other : v;
        }
        return v;
    }
}

template <bool Shuffle = false>
__device__ __forceinline__ WarpHeadSegments warp_head_segments(int is_head, int lane_id)
{
    // tail flags of this lane and above, plus the warp's last lane
    unsigned tails = __ballot_sync(0xffffffffu, is_head) >> 1;
    tails &= 0xffffffffu << lane_id;
    tails |= 1u << 31;
    const int last_lane = __ffs(static_cast<int>(tails)) - 1;
    return {last_lane, static_cast<int>(warp_max<Shuffle>(last_lane - lane_id + 1))};
}

template <int BlockSize = 128, int WarpSize = 32>
class FastSegmentalReduce : public LaunchBase<FastSegmentalReduce<BlockSize, WarpSize>>
{
    using Base = LaunchBase<FastSegmentalReduce<BlockSize, WarpSize>>;

  public:
    // Lanes per warp: a warp reduces WarpSize consecutive inputs (warps are
    // aligned at multiples of WarpSize). Hardware warps only.
    static_assert(WarpSize == 32, "FastSegmentalReduce reduces hardware warps");
    static constexpr int warp_size = WarpSize;

    FastSegmentalReduce(cudaStream_t s = nullptr)
        : Base(s)
    {
    }

    // e.g.
    // when ReduceOp = ::cuda::std::plus
    // dst = [0, 1, 1, 2, 2, 2]
    // in  = [1, 1, 1, 1, 1, 1]
    // out = [1, 2, 3]
    template <typename T, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce(CBufferView<int> dst,
                                CBufferView<T>   in,
                                BufferView<T>    out,
                                ReduceOp         op = ReduceOp{});

    template <typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce(size_t        in_size,
                                BufferView<T> out,
                                GetKeyOp      get_key_op,
                                GetValueOp    get_value_op,
                                ReduceOp      op = ReduceOp{});


    template <typename T, int M, int N, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce(CBufferView<int>                    dst,
                                CBufferView<Eigen::Matrix<T, M, N>> in,
                                BufferView<Eigen::Matrix<T, M, N>>  out,
                                ReduceOp op = ReduceOp{});

    template <typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce(size_t                             in_size,
                                BufferView<Eigen::Matrix<T, M, N>> out,
                                GetKeyOp                           get_key_op,
                                GetValueOp                         get_value_op,
                                ReduceOp op = ReduceOp{});

    // reduce() without clearing `out` first. The head of a segment that lies
    // within one warp stores the segment's result; every piece of a segment
    // that spans warps adds its partial result atomically, and so does the
    // last segment when in_size is not a multiple of WarpSize (the lanes past
    // in_size join it and mark it as continuing). The caller must have
    // cleared (set to the identity of op) exactly those outputs.
    template <typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce_uncleared(size_t        in_size,
                                          BufferView<T> out,
                                          GetKeyOp      get_key_op,
                                          GetValueOp    get_value_op,
                                          ReduceOp      op = ReduceOp{});

    template <typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp = ::cuda::std::plus<T>>
    FastSegmentalReduce& reduce_uncleared(size_t in_size,
                                          BufferView<Eigen::Matrix<T, M, N>> out,
                                          GetKeyOp   get_key_op,
                                          GetValueOp get_value_op,
                                          ReduceOp   op = ReduceOp{});
};
}  // namespace uipc::backend::cuda_tool

#include "details/fast_segmental_reduce.inl"
