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

// cuda_tool::SideStream, one non-blocking stream beside a parent stream:
// fork() orders it after the work enqueued on the parent so far and join()
// orders the parent after the work enqueued on it.
// Slow kernels make a missing dependency visible as a wrong value.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>

#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
__device__ void side_stream_test_spin(long long cycles)
{
    long long start = clock64();
    while(clock64() - start < cycles)
    {
    }
}

__global__ void side_stream_test_fill_kernel(int* dst, int value, int n, long long spin)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    side_stream_test_spin(spin);
    dst[i] = value;
}

__global__ void side_stream_test_scale_kernel(const int* src, int* dst, int factor, int n, long long spin)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    side_stream_test_spin(spin);
    dst[i] = factor * src[i];
}

constexpr int       N          = 1024;
constexpr int       BlockDim   = 128;
constexpr int       GridDim    = (N + BlockDim - 1) / BlockDim;
constexpr long long SpinCycles = 200000;

std::vector<int> host_of(const cuda_tool::DeviceBuffer<int>& d)
{
    std::vector<int> h;
    d.copy_to(h);
    return h;
}
}  // namespace

TEST_CASE("cuda_tool SideStream forks from its parent and joins it", "[cuda][cuda_tool][stream]")
{
    cuda_tool::SideStream        side;
    cuda_tool::DeviceBuffer<int> a(N);
    cuda_tool::DeviceBuffer<int> b(N);

    for(int round = 1; round <= 3; ++round)
    {
        // fork(): the side stream reads what the parent wrote before it
        side_stream_test_fill_kernel<<<GridDim, BlockDim>>>(a.data(), round, N, SpinCycles);
        cudaStream_t s = side.fork();
        REQUIRE(s != nullptr);
        side_stream_test_scale_kernel<<<GridDim, BlockDim, 0, s>>>(
            a.data(), b.data(), 3, N, SpinCycles);
        // join(): the parent reads what the side stream wrote
        side.join();
        auto h = host_of(b);
        for(int i = 0; i < N; ++i)
            REQUIRE(h[i] == 3 * round);
        REQUIRE(side.fork() == s);  // one stream, created on first use
        side.join();
    }
}
