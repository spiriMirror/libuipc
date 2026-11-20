/*
 * NVIDIA contributions: ELF hidden visibility on kernel launch helpers to
 * prevent cross-module interposition.
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
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>
#include <unordered_map>

namespace uipc::backend::cuda_tool
{
// Best occupancy block size for a kernel (cached per kernel symbol), matching
// what the old ParallelFor's automatic selection would pick. Named-kernel
// launch sites use this to keep the same FP behavior as the lambda
// ParallelFor they replaced.
//
// The cache is keyed by kernel address: distinct kernels may share the same
// function-pointer type (same signature), so keying by the template
// parameter would let one kernel's block size leak into another's.
template <typename Kernel>
#if defined(__ELF__)
[[gnu::visibility("hidden")]]
#endif
int best_block_dim(Kernel kernel, size_t shared_mem_size = 0)
{
    static thread_local std::unordered_map<const void*, int> cached_block_sizes;
    const void* key = (const void*)kernel;
    auto        it  = cached_block_sizes.find(key);
    if(it == cached_block_sizes.end())
    {
        int min_grid_size = -1;
        int block_size    = -1;
        CUDA_TOOL_CHECK(cudaOccupancyMaxPotentialBlockSize(
            &min_grid_size, &block_size, kernel, shared_mem_size));
        it = cached_block_sizes.emplace(key, block_size).first;
    }
    return it->second;
}

// grid dim for launching `kernel` over n items with best_block_dim(kernel)
template <typename Kernel>
#if defined(__ELF__)
[[gnu::visibility("hidden")]]
#endif
int best_grid_dim(int n, Kernel kernel, size_t shared_mem_size = 0)
{
    int bd = best_block_dim(kernel, shared_mem_size);
    return (n + bd - 1) / bd;
}
}  // namespace uipc::backend::cuda_tool
