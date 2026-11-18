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

// cuda_tool's CUB wrappers run on the stream's cached workspace: a call that
// fits runs once, a call that needs more temporary storage than the
// workspace holds (CUB returns cudaErrorInvalidValue before it launches
// anything) grows the workspace and runs again; either way the result is
// CUB's.

#include <app/app.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;

TEST_CASE("cuda_tool CUB wrappers grow the stream's cached workspace", "[cuda][cuda_tool][cub]")
{
    cudaStream_t s = nullptr;
    CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));
    const auto& workspace = cuda_tool::details::cub_workspace(s);

    size_t last_capacity = 0;
    for(int n : {16, 1 << 12, 1 << 22, 64})
    {
        std::vector<std::int64_t> h(n);
        std::iota(h.begin(), h.end(), std::int64_t{1});
        cuda_tool::DeviceBuffer<std::int64_t> in;
        in.resize_discard(n, s);
        CUDA_TOOL_CHECK(cudaMemcpyAsync(
            in.data(), h.data(), n * sizeof(std::int64_t), cudaMemcpyHostToDevice, s));

        cuda_tool::DeviceVar<std::int64_t>    sum;
        cuda_tool::DeviceBuffer<std::int64_t> scan;
        scan.resize_discard(n, s);
        cuda_tool::DeviceReduce(s).Sum(in.data(), sum.data(), n);
        cuda_tool::DeviceScan(s).InclusiveSum(in.data(), scan.data(), n);

        std::int64_t h_sum = 0, h_last = 0;
        CUDA_TOOL_CHECK(cudaMemcpyAsync(
            &h_sum, sum.data(), sizeof(h_sum), cudaMemcpyDeviceToHost, s));
        CUDA_TOOL_CHECK(cudaMemcpyAsync(
            &h_last, scan.data() + n - 1, sizeof(h_last), cudaMemcpyDeviceToHost, s));
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        const std::int64_t expect = std::int64_t(n) * (n + 1) / 2;
        REQUIRE(h_sum == expect);
        REQUIRE(h_last == expect);

        // the workspace only grows; the last, small calls fit and keep it
        REQUIRE(workspace.capacity() > 0);
        REQUIRE(workspace.capacity() >= last_capacity);
        if(n == 64)
            REQUIRE(workspace.capacity() == last_capacity);
        last_capacity = workspace.capacity();
    }

    CUDA_TOOL_CHECK(cudaStreamDestroy(s));
}
