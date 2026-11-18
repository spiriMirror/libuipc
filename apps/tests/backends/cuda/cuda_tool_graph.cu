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

// cuda_tool::GraphCapture and cuda_tool::GraphWhile when a capture body
// fails. A body that throws on a valid capture propagates its exception
// once the capture is ended, and the instance stays usable. A body that
// escapes the capture stream (a launch into the legacy stream invalidates
// the capture) fails the capture whether a later checked call then throws
// or not, and the instance is disabled; GraphWhile abandons the parent
// graph of a failed loop-body capture instead of destroying it.
// GraphWhile::launch() enqueues the loop on its blocking stream without a
// host wait.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/graph.h>

#include <stdexcept>

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
__global__ void graph_test_store_kernel(int* out, int value)
{
    *out = value;
}

__global__ void graph_test_noop_kernel() {}

#if CUDA_TOOL_GRAPH_WHILE
__global__ void graph_test_reset_kernel(int* counter)
{
    *counter = 0;
}

// One loop iteration: count, and continue while the count is below `limit`.
__global__ void graph_test_step_kernel(int* counter, int limit, cudaGraphConditionalHandle handle)
{
    const int k = ++(*counter);
    cudaGraphSetConditional(handle, k < limit ? 1u : 0u);
}

__global__ void graph_test_copy_kernel(const int* src, int* dst)
{
    *dst = *src;
}
#endif

int read_back(const int* d)
{
    int h = 0;
    CUDA_TOOL_CHECK(cudaMemcpy(&h, d, sizeof(int), cudaMemcpyDeviceToHost));
    return h;
}

// A launch into the legacy default stream invalidates the capture of `s`;
// the checked event record on `s` that follows then fails and throws.
void escape_and_throw(cudaStream_t s, cudaEvent_t event)
{
    graph_test_noop_kernel<<<1, 1, 0, cudaStreamLegacy>>>();
    CUDA_TOOL_CHECK(cudaEventRecord(event, s));
}
}  // namespace

TEST_CASE("cuda_tool GraphCapture ends a capture whose body throws", "[cuda][graph]")
{
    using Result = cuda_tool::GraphCapture::Result;
    int* d       = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d, sizeof(int)));
    cudaEvent_t event = nullptr;
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&event, cudaEventDisableTiming));
    auto store = [d](int value)
    {
        return [d, value](cudaStream_t s)
        { graph_test_store_kernel<<<1, 1, 0, s>>>(d, value); };
    };

    SECTION("a body that throws on a valid capture")
    {
        cuda_tool::GraphCapture g;
        REQUIRE_THROWS_AS(g.capture(
                              [d](cudaStream_t s)
                              {
                                  graph_test_store_kernel<<<1, 1, 0, s>>>(d, 9);
                                  throw std::runtime_error("capture body");
                              }),
                          std::runtime_error);
        REQUIRE_FALSE(g.ready());
        REQUIRE_FALSE(g.disabled());
        // the capture stream left capture mode: the next capture works
        REQUIRE(g.capture(store(7)) == Result::Ok);
        g.launch_sync();
        REQUIRE(read_back(d) == 7);
    }

    SECTION("a body that throws after escaping the capture stream")
    {
        // reported like an invalidation that cudaStreamEndCapture detects
        cuda_tool::GraphCapture g;
        REQUIRE(g.capture([event](cudaStream_t s) { escape_and_throw(s, event); })
                == Result::CaptureInvalidated);
        REQUIRE_FALSE(g.ready());
        REQUIRE(g.disabled());
        // the escaped launch's error is pending in this executable's runtime
        cudaGetLastError();
        // no stream was left capturing: another instance captures and replays
        cuda_tool::GraphCapture next;
        REQUIRE(next.capture(store(5)) == Result::Ok);
        next.launch_sync();
        REQUIRE(read_back(d) == 5);
    }

    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    CUDA_TOOL_CHECK(cudaEventDestroy(event));
    CUDA_TOOL_CHECK(cudaFree(d));
}

TEST_CASE("cuda_tool GraphWhile disables itself when its loop body capture fails", "[cuda][graph]")
{
#if CUDA_TOOL_GRAPH_WHILE
    if(!cuda_tool::GraphWhile::runtime_supported())
    {
        WARN("conditional graph nodes need a CUDA 12.4 driver");
        return;
    }
    using Result = cuda_tool::GraphWhile::Result;

    int* counter = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&counter, sizeof(int)));
    cudaEvent_t event = nullptr;
    CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&event, cudaEventDisableTiming));
    auto setup = [counter](cudaStream_t s, cudaGraphConditionalHandle)
    { graph_test_reset_kernel<<<1, 1, 0, s>>>(counter); };
    auto body = [counter](cudaStream_t s, cudaGraphConditionalHandle handle)
    { graph_test_step_kernel<<<1, 1, 0, s>>>(counter, 3, handle); };

    SECTION("a body that escapes the capture stream")
    {
        cuda_tool::GraphWhile g;
        REQUIRE(g.capture(setup,
                          [](cudaStream_t, cudaGraphConditionalHandle) {
                              graph_test_noop_kernel<<<1, 1, 0, cudaStreamLegacy>>>();
                          })
                == Result::CaptureFailed);
        REQUIRE_FALSE(g.ready());
        REQUIRE(g.disabled());
        // a disabled instance keeps its failure and does not capture again
        REQUIRE(g.capture(setup, body) == Result::CaptureFailed);
        REQUIRE_FALSE(g.ready());
    }

    SECTION("a body that throws after escaping the capture stream")
    {
        cuda_tool::GraphWhile g;
        REQUIRE(g.capture(setup,
                          [event](cudaStream_t s, cudaGraphConditionalHandle)
                          { escape_and_throw(s, event); })
                == Result::CaptureFailed);
        REQUIRE_FALSE(g.ready());
        REQUIRE(g.disabled());
    }

    // the escaped launches' errors are pending in this executable's runtime;
    // the abandoned graphs are never destroyed
    cudaGetLastError();
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    CUDA_TOOL_CHECK(cudaEventDestroy(event));
    CUDA_TOOL_CHECK(cudaFree(counter));
#endif
}

TEST_CASE("cuda_tool GraphWhile::launch enqueues the loop without a host wait", "[cuda][graph]")
{
#if CUDA_TOOL_GRAPH_WHILE
    if(!cuda_tool::GraphWhile::runtime_supported())
    {
        WARN("conditional graph nodes need a CUDA 12.4 driver");
        return;
    }

    int* counter = nullptr;
    int* copy    = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&counter, sizeof(int)));
    CUDA_TOOL_CHECK(cudaMalloc(&copy, sizeof(int)));

    for(int limit : {1, 5, 37})
    {
        cuda_tool::GraphWhile g;
        REQUIRE(g.launch_stream() == nullptr);
        auto setup = [counter](cudaStream_t s, cudaGraphConditionalHandle)
        { graph_test_reset_kernel<<<1, 1, 0, s>>>(counter); };
        auto body = [counter, limit](cudaStream_t s, cudaGraphConditionalHandle handle)
        { graph_test_step_kernel<<<1, 1, 0, s>>>(counter, limit, handle); };
        REQUIRE(g.capture(setup, body) == cuda_tool::GraphWhile::Result::Ok);
        REQUIRE(g.ready());

        // twice, the second time over the first launch's result: the setup
        // node resets the counter on every launch
        for(int rep = 0; rep < 2; ++rep)
        {
            CUDA_TOOL_CHECK(g.launch());
            REQUIRE(g.launch_stream() != nullptr);
            // legacy-stream work enqueued after launch() runs after the graph
            // (the launch stream is blocking)
            graph_test_copy_kernel<<<1, 1>>>(counter, copy);
            REQUIRE(read_back(copy) == limit);
        }

        CUDA_TOOL_CHECK(g.launch_sync());
        REQUIRE(read_back(counter) == limit);
    }

    CUDA_TOOL_CHECK(cudaFree(copy));
    CUDA_TOOL_CHECK(cudaFree(counter));
#endif
}
