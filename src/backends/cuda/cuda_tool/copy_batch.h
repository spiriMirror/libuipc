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
#include <cuda_tool/buffer.h>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace uipc::backend::cuda_tool
{
namespace details
{
    // The ranges of a CopyBatch, in 8-byte words. Range r covers the words
    // [begin[r], begin[r + 1]) of the batch; src[r] == nullptr zero-fills.
    inline constexpr int CopyBatchCapacity = 16;
    struct CopyBatchRanges
    {
        const std::uint64_t* src[CopyBatchCapacity];
        std::uint64_t*       dst[CopyBatchCapacity];
        std::uint64_t        begin[CopyBatchCapacity + 1];
        int                  count;
    };

    // A template so that only the translation units that launch a batch
    // compile it.
    template <int Capacity>
    __global__ void copy_batch_kernel(CopyBatchRanges ranges)
    {
        static_assert(Capacity == CopyBatchCapacity);
        const std::uint64_t total  = ranges.begin[ranges.count];
        const std::uint64_t stride = (std::uint64_t)gridDim.x * blockDim.x;
        for(std::uint64_t w = (std::uint64_t)blockIdx.x * blockDim.x + threadIdx.x; w < total;
            w += stride)
        {
            int r = ranges.count - 1;
            while(w < ranges.begin[r])
                --r;
            const std::uint64_t i = w - ranges.begin[r];
            ranges.dst[r][i] = ranges.src[r] ? ranges.src[r][i] : std::uint64_t{0};
        }
    }
}  // namespace details

// Device-to-device copies and zero fills of ranges that do not overlap,
// collected by copy() and zero() and enqueued by launch() as one kernel
// instead of one copy, memset or kernel each: the same bytes, one launch.
// A range that is not made of whole 8-byte words gets its own
// cudaMemcpyAsync or cudaMemsetAsync in launch().
class CopyBatch
{
  public:
    // dst = src (sizes must match)
    template <typename T>
    void copy(BufferView<T> dst, CBufferView<T> src)
    {
        if(dst.size() != src.size())
            throw std::runtime_error{"CopyBatch::copy size mismatch"};
        add(dst.data(), src.data(), dst.size() * sizeof(T));
    }
    // every byte of dst = 0
    template <typename T>
    void zero(BufferView<T> dst)
    {
        add(dst.data(), nullptr, dst.size() * sizeof(T));
    }

    // Enqueue the collected ranges on `s` and empty the batch. (A member
    // template: only its callers compile the kernel.)
    template <typename = void>
    void launch(cudaStream_t s = default_stream())
    {
        details::CopyBatchRanges ranges{};
        for(int r = 0; r < m_count; ++r)
        {
            const Range& x = m_ranges[r];
            if(x.bytes % sizeof(std::uint64_t) == 0
               && reinterpret_cast<std::uintptr_t>(x.dst) % alignof(std::uint64_t) == 0
               && reinterpret_cast<std::uintptr_t>(x.src) % alignof(std::uint64_t) == 0)
            {
                int k         = ranges.count++;
                ranges.dst[k] = static_cast<std::uint64_t*>(x.dst);
                ranges.src[k] = static_cast<const std::uint64_t*>(x.src);
                ranges.begin[k + 1] = ranges.begin[k] + x.bytes / sizeof(std::uint64_t);
            }
            else if(x.src)
                CUDA_TOOL_CHECK(cudaMemcpyAsync(x.dst, x.src, x.bytes, cudaMemcpyDeviceToDevice, s));
            else
                CUDA_TOOL_CHECK(cudaMemsetAsync(x.dst, 0, x.bytes, s));
        }
        m_count = 0;

        const std::uint64_t words = ranges.begin[ranges.count];
        if(words == 0)
            return;
        const std::uint64_t blocks = (words + default_block_dim - 1) / default_block_dim;
        const int grid =
            (int)std::min<std::uint64_t>(blocks, std::numeric_limits<int>::max());
        details::copy_batch_kernel<details::CopyBatchCapacity>
            <<<grid, default_block_dim, 0, s>>>(ranges);
    }

    bool empty() const noexcept { return m_count == 0; }

  private:
    struct Range
    {
        void*       dst   = nullptr;
        const void* src   = nullptr;
        size_t      bytes = 0;
    };

    void add(void* dst, const void* src, size_t bytes)
    {
        if(bytes == 0)
            return;
        if(m_count == details::CopyBatchCapacity)
            throw std::length_error{"CopyBatch: too many ranges"};
        m_ranges[m_count++] = Range{dst, src, bytes};
    }

    Range m_ranges[details::CopyBatchCapacity];
    int   m_count = 0;
};
}  // namespace uipc::backend::cuda_tool
