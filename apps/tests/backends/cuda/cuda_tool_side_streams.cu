/*
 * NVIDIA contributions: fork/join ordering tests for cuda_tool side streams.
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

#include <cuda_tool/cuda_tool.h>

#include <app/app.h>

#include <vector>

using uipc::backend::cuda_tool::DeviceBuffer;
using uipc::backend::cuda_tool::LazySideStream;
using uipc::backend::cuda_tool::SideStreams;

namespace
{
// Busy-wait so a missing dependency lets the next kernel overtake this one.
__device__ void side_streams_test_spin(long long cycles)
{
    long long start = clock64();
    while(clock64() - start < cycles)
    {
    }
}

__global__ void side_streams_test_fill_kernel(int* dst, int value, int n, long long spin)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    side_streams_test_spin(spin);
    dst[i] = value;
}

__global__ void side_streams_test_scale_kernel(const int* src, int* dst, int factor, int n, long long spin)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    side_streams_test_spin(spin);
    dst[i] = factor * src[i];
}

__global__ void side_streams_test_add_kernel(const int* src, int* dst, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    dst[i] += src[i];
}

constexpr int       N          = 1024;
constexpr int       BlockDim   = 128;
constexpr int       GridDim    = (N + BlockDim - 1) / BlockDim;
constexpr long long SpinCycles = 200000;
}  // namespace

TEST_CASE("cuda_tool SideStreams order forked producers", "[cuda][cuda_tool][stream]")
{
    SideStreams side;
    side.create(4);
    REQUIRE(side.size() == 4);

    DeviceBuffer<int> src(N);
    DeviceBuffer<int> dst(4 * N);

    for(int round = 1; round <= 4; ++round)
    {
        // legacy stream: slow producer of the shared input
        side_streams_test_fill_kernel<<<GridDim, BlockDim>>>(src.data(), round, N, SpinCycles);

        // each side stream sees the input, then writes its own range
        side.fork();
        for(int i = 0; i < 4; ++i)
        {
            cudaStream_t s = side.next();
            REQUIRE(s == side[i]);
            side_streams_test_scale_kernel<<<GridDim, BlockDim, 0, s>>>(
                src.data(), dst.data() + i * N, i + 1, N, SpinCycles);
        }

        // the legacy stream reads every range only after the join
        side.join();
        std::vector<int> host;
        dst.copy_to(host);
        REQUIRE(host.size() == 4 * N);
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < N; ++j)
                REQUIRE(host[i * N + j] == (i + 1) * round);
    }
}

TEST_CASE("cuda_tool SideStreams gather on a side stream", "[cuda][cuda_tool][stream]")
{
    SideStreams side;
    side.create(3);

    DeviceBuffer<int> base(N);
    DeviceBuffer<int> part(2 * N);

    for(int round = 1; round <= 4; ++round)
    {
        side.fork();
        // producers on side streams 1 and 2
        side_streams_test_fill_kernel<<<GridDim, BlockDim, 0, side[1]>>>(
            part.data(), 10 * round, N, SpinCycles);
        side_streams_test_fill_kernel<<<GridDim, BlockDim, 0, side[2]>>>(
            part.data() + N, 100 * round, N, SpinCycles);
        // plain stores on the legacy stream
        side_streams_test_fill_kernel<<<GridDim, BlockDim>>>(base.data(), round, N, SpinCycles);

        // the gather on side stream 0 waits for the legacy stream and the
        // other side streams, and the legacy stream joins it
        side.fork_one(0);
        side.join(side[0]);
        side_streams_test_add_kernel<<<GridDim, BlockDim, 0, side[0]>>>(
            part.data(), base.data(), N);
        side_streams_test_add_kernel<<<GridDim, BlockDim, 0, side[0]>>>(
            part.data() + N, base.data(), N);
        side.join();

        std::vector<int> host;
        base.copy_to(host);
        for(int j = 0; j < N; ++j)
            REQUIRE(host[j] == round + 10 * round + 100 * round);
    }
}

TEST_CASE("cuda_tool LazySideStream takes a stream on first use", "[cuda][cuda_tool][stream]")
{
    SideStreams side;
    side.create(2);
    side.fork();

    LazySideStream unused{&side};
    LazySideStream first{&side};
    LazySideStream second{&side};
    LazySideStream legacy;

    REQUIRE(first.get() == side[0]);
    REQUIRE(first.get() == side[0]);
    REQUIRE(second.get() == side[1]);
    REQUIRE(legacy.get() == nullptr);

    SideStreams none;
    REQUIRE(none.size() == 0);
    none.fork();
    REQUIRE(none.next() == nullptr);
    none.join();
}

TEST_CASE("cuda_tool SideStreams::keep_handed_out keeps the streams of long chains",
          "[cuda][cuda_tool][stream]")
{
    SideStreams side;
    side.create(4);
    side.fork();

    // two long chains take streams 0 and 1 and keep them
    REQUIRE(side.next() == side[0]);
    REQUIRE(side.next() == side[1]);
    side.keep_handed_out();
    // the other producers cycle over streams 2 and 3 only
    REQUIRE(side.next() == side[2]);
    REQUIRE(side.next() == side[3]);
    REQUIRE(side.next() == side[2]);
    side.join();

    // the next fork() hands out every stream again
    side.fork();
    for(int i = 0; i < 4; ++i)
        REQUIRE(side.next() == side[i]);
    side.join();

    // with every stream kept, next() cycles over all of them
    side.fork();
    for(int i = 0; i < 4; ++i)
        side.next();
    side.keep_handed_out();
    REQUIRE(side.next() == side[0]);
    REQUIRE(side.next() == side[1]);
    side.join();
}
