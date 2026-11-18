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

// cuda_tool::ReadbackBuffer on its own: the host reads the values a kernel
// stored through device_data() after one synchronization of the producing
// stream, on the mapped path (when the device maps page-locked host memory)
// and on the copy path; copy_from() reads device memory another producer
// wrote.
// A new allocation moves the device allocation epoch.
// store_value() puts a host value into device memory or into a read-back
// slot.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>

#include <stdexcept>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;

namespace
{
__global__ void readback_test_iota_kernel(int* out, int n, int base)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < n)
        out[i] = base + i;
}

__global__ void readback_test_copy_kernel(const double* src, double* dst)
{
    *dst = *src;
}

constexpr cuda_tool::ReadbackPath Paths[] = {cuda_tool::ReadbackPath::Auto,
                                             cuda_tool::ReadbackPath::Copy};
}  // namespace

TEST_CASE("cuda_tool ReadbackBuffer returns a kernel's stores after one synchronization",
          "[cuda][cuda_tool][readback]")
{
    cudaStream_t s = nullptr;
    CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));

    for(auto path : Paths)
    {
        cuda_tool::ReadbackBuffer<int> rb{path};
        REQUIRE(rb.size() == 0);

        // every round grows the buffer: the allocation (and the device
        // address the producer stores to) is new, so the epoch moves
        for(int round = 0; round < 3; ++round)
        {
            const int  n      = 4 + 300 * round;
            const auto before = cuda_tool::device_allocation_epoch();
            rb.resize_discard(n);
            REQUIRE(rb.size() == static_cast<size_t>(n));
            REQUIRE(cuda_tool::device_allocation_epoch() > before);

            readback_test_iota_kernel<<<(n + 127) / 128, 128, 0, s>>>(
                rb.device_data(), n, 1000 * round);
            rb.enqueue_copy(s);
            CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
            for(int i = 0; i < n; ++i)
                REQUIRE(rb[i] == 1000 * round + i);
        }

        const bool mapped = path == cuda_tool::ReadbackPath::Auto
                            && cuda_tool::mapped_readbacks();
        REQUIRE(rb.mapped() == mapped);
        // on the mapped path the producer stores into host memory directly
        if(mapped)
            REQUIRE(static_cast<void*>(rb.device_data()) == static_cast<void*>(rb.data()));

        // a smaller size keeps the allocation
        const auto before = cuda_tool::device_allocation_epoch();
        rb.resize_discard(2);
        REQUIRE(rb.size() == 2);
        REQUIRE(cuda_tool::device_allocation_epoch() == before);
        readback_test_iota_kernel<<<1, 32, 0, s>>>(rb.device_data(), 2, 7);
        rb.enqueue_copy(s);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        REQUIRE(rb[0] == 7);
        REQUIRE(rb[1] == 8);
    }

    CUDA_TOOL_CHECK(cudaStreamDestroy(s));
}

TEST_CASE("cuda_tool ReadbackBuffer::copy_from reads device memory another producer wrote",
          "[cuda][cuda_tool][readback]")
{
    cuda_tool::DeviceBuffer<int> src(5);
    readback_test_iota_kernel<<<1, 32>>>(src.data(), 5, 40);

    for(auto path : Paths)
    {
        cuda_tool::ReadbackBuffer<int> rb{path};
        rb.resize_discard(8);
        rb.copy_from(3, src.cview());
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
        for(int i = 0; i < 5; ++i)
            REQUIRE(rb[3 + i] == 40 + i);
        REQUIRE_THROWS_AS(rb.copy_from(4, src.cview()), std::length_error);
    }
}

TEST_CASE("cuda_tool store_value writes a host value into device memory and read-back slots",
          "[cuda][cuda_tool][readback]")
{
    cuda_tool::DeviceBuffer<double> d(1);
    cuda_tool::store_value(d.data(), 2.5);
    std::vector<double> h;
    d.copy_to(h);
    REQUIRE(h.size() == 1);
    REQUIRE(h[0] == 2.5);

    cudaStream_t s = nullptr;
    CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));
    for(auto path : Paths)
    {
        cuda_tool::ReadbackBuffer<double> rb{path};
        rb.resize_discard(2);
        // slot 1 from the host, slot 0 from a kernel, read back together
        cuda_tool::store_value(rb.device_data() + 1, 4.25, s);
        readback_test_copy_kernel<<<1, 1, 0, s>>>(d.data(), rb.device_data());
        rb.enqueue_copy(s);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        REQUIRE(rb[0] == 2.5);
        REQUIRE(rb[1] == 4.25);
    }
    CUDA_TOOL_CHECK(cudaStreamDestroy(s));
}
