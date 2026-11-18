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

// cuda_tool::GraphCapture::recapture() (the line search's cached energy
// graph) against capture(): a new capture replays its own launches, whether
// it updates the instantiated graph or instantiates a new one, and every
// failure leaves the instance as capture() leaves it.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/graph.h>

#include <stdexcept>

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
using Result = cuda_tool::GraphCapture::Result;

__global__ void graph_capture_test_store_kernel(int* out, int value)
{
    *out = value;
}

int read_back(const int* d)
{
    int h = 0;
    CUDA_TOOL_CHECK(cudaMemcpy(&h, d, sizeof(int), cudaMemcpyDeviceToHost));
    return h;
}

auto store(int* d, int value)
{
    return [d, value](cudaStream_t s)
    { graph_capture_test_store_kernel<<<1, 1, 0, s>>>(d, value); };
}
}  // namespace

TEST_CASE("GraphCapture recapture replays the launches of the new capture", "[cuda][graph]")
{
    int* d = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d, sizeof(int)));

    cuda_tool::GraphCapture g;
    REQUIRE(g.recapture(store(d, 1)) == Result::Ok);  // nothing instantiated: capture()
    g.launch_sync();
    REQUIRE(read_back(d) == 1);

    REQUIRE(g.recapture(store(d, 2)) == Result::Ok);  // same topology: updated in place
    g.launch_sync();
    REQUIRE(read_back(d) == 2);

    REQUIRE(g.recapture(
                [d](cudaStream_t s)
                {
                    graph_capture_test_store_kernel<<<1, 1, 0, s>>>(d, 3);
                    graph_capture_test_store_kernel<<<1, 1, 0, s>>>(d, 4);
                })
            == Result::Ok);  // new topology: instantiated again
    g.launch_sync();
    REQUIRE(read_back(d) == 4);

    cudaStream_t s = nullptr;
    CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));
    REQUIRE(g.recapture(store(d, 5)) == Result::Ok);
    CUDA_TOOL_CHECK(g.launch(s));
    CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    REQUIRE(read_back(d) == 5);
    REQUIRE(g.ready());
    REQUIRE_FALSE(g.disabled());

    CUDA_TOOL_CHECK(cudaStreamDestroy(s));
    CUDA_TOOL_CHECK(cudaFree(d));
}

TEST_CASE("GraphCapture recapture fails like capture", "[cuda][graph]")
{
    int* d = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d, sizeof(int)));
    auto throwing = [d](cudaStream_t s)
    {
        graph_capture_test_store_kernel<<<1, 1, 0, s>>>(d, 9);
        throw std::runtime_error("capture body");
    };

    SECTION("a body that throws on a valid capture")
    {
        // capture(): the exception propagates; nothing is instantiated and
        // the instance stays usable.
        cuda_tool::GraphCapture first;
        REQUIRE_THROWS_AS(first.capture(throwing), std::runtime_error);
        REQUIRE_FALSE(first.ready());
        REQUIRE_FALSE(first.disabled());

        // recapture() of an instantiated graph: the same state, so the
        // former graph is never replayed for the failed capture.
        cuda_tool::GraphCapture g;
        REQUIRE(g.capture(store(d, 1)) == Result::Ok);
        REQUIRE_THROWS_AS(g.recapture(throwing), std::runtime_error);
        REQUIRE_FALSE(g.ready());
        REQUIRE_FALSE(g.disabled());

        // Both capture streams left capture mode: the next capture works.
        for(auto* x : {&first, &g})
        {
            REQUIRE(x->recapture(store(d, 7)) == Result::Ok);
            x->launch_sync();
            REQUIRE(read_back(d) == 7);
        }
    }

    SECTION("a body that escapes the capture stream")
    {
        // A launch into the legacy stream invalidates the capture: the
        // instance is disabled for good, as capture() does.
        cuda_tool::GraphCapture g;
        REQUIRE(g.capture(store(d, 1)) == Result::Ok);
        REQUIRE(g.recapture(
                    [d](cudaStream_t) {
                        graph_capture_test_store_kernel<<<1, 1, 0, cudaStreamLegacy>>>(d, 8);
                    })
                == Result::CaptureInvalidated);
        REQUIRE_FALSE(g.ready());
        REQUIRE(g.disabled());
        REQUIRE(g.recapture(store(d, 2)) == Result::CaptureInvalidated);
        REQUIRE_FALSE(g.ready());
        // The escaped launch's error is pending in this executable's runtime.
        cudaGetLastError();
    }

    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    CUDA_TOOL_CHECK(cudaFree(d));
}
