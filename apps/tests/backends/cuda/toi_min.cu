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

// The trajectory filters take the minimum of their TOIs inside the TOI
// kernels (toi_min.h) instead of a DeviceReduce::Min launch after them. This
// case requires the bytes of DeviceReduce::Min's result over the same values
// (positive TOIs, the "no hit" value 1.1 many times over, negative half-plane
// TOIs, +inf), split over one to four launches of one-warp or larger blocks
// with empty launches in between, and the slot reset after every use.

#include <app/app.h>
#include <collision_detection/filters/toi_min.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace toi_min_test
{
__global__ void toi_min_test_kernel(const double* v, int n, TOIMinSlot slot, double* out)
{
    int                i   = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned long long key = TOIMinIdentity;
    if(i < n)
        key = toi_min_key(v[i]);
    toi_min_block(key, slot, out);
}
}  // namespace toi_min_test

TEST_CASE("TOI minimum inside the TOI kernels equals DeviceReduce::Min",
          "[cuda][collision detection][toi min]")
{
    std::mt19937_64                        rng{29};
    std::uniform_real_distribution<double> unit{0.0, 1.0};

    cuda_tool::DeviceBuffer<unsigned long long> slot;
    const unsigned long long                    init[2] = {TOIMinIdentity, 0};
    slot.copy_from(init, 2);
    cuda_tool::DeviceBuffer<double> out, ref;
    out.resize(1);
    ref.resize(1);

    auto run = [&](std::vector<int> sizes, int block, int kind)
    {
        int total = 0;
        for(int n : sizes)
            total += n;
        std::vector<double> v(total);
        for(auto& x : v)
        {
            double u = unit(rng);
            switch(kind)
            {
                case 0:  // CCD TOIs: mostly "no hit"
                    x = u < 0.9 ? 1.1 : unit(rng);
                    break;
                case 1:  // half-plane TOIs: negative inside the thickness
                    x = u < 0.5 ? 1.1f : (unit(rng) - 0.3) * 4.0;
                    break;
                case 2:  // few distinct values, many ties
                    x = 0.25 * static_cast<int>(u * 5.0);
                    x = x == 0.0 ? 1.1 : x;
                    break;
                default:  // anything finite or +inf
                    x = u < 0.01 ? std::numeric_limits<double>::infinity() :
                                   (unit(rng) - 0.5) * 1e6;
                    break;
            }
        }
        cuda_tool::DeviceBuffer<double> d;
        d.copy_from(v.data(), v.size());
        cuda_tool::DeviceReduce().Min(d.data(), ref.data(), total);

        int last = -1;
        for(int l = 0; l < static_cast<int>(sizes.size()); ++l)
            if(sizes[l] > 0)
                last = l;
        int offset = 0;
        for(int l = 0; l < static_cast<int>(sizes.size()); ++l)
        {
            int n = sizes[l];
            if(n > 0)
            {
                int blocks = (n + block - 1) / block;
                toi_min_test::toi_min_test_kernel<<<blocks, block>>>(
                    d.data() + offset,
                    n,
                    TOIMinSlot{slot.data(), slot.data() + 1},
                    l == last ? out.data() : nullptr);
            }
            offset += n;
        }
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);

        double a = 0, b = 0;
        out.copy_to(&a);
        ref.copy_to(&b);
        REQUIRE(std::memcmp(&a, &b, sizeof(double)) == 0);
        std::vector<unsigned long long> s;
        slot.copy_to(s);
        REQUIRE(s[0] == TOIMinIdentity);
        REQUIRE(s[1] == 0);
    };
    for(int kind = 0; kind < 4; ++kind)
    {
        for(int block : {32, 256})
        {
            run({1}, block, kind);
            run({31}, block, kind);
            run({32}, block, kind);
            run({33}, block, kind);
            run({1000}, block, kind);
            run({0, 0, 5000, 70000}, block, kind);  // PT and EE
            run({0, 0, 5000, 0}, block, kind);  // the finishing launch is not the last list
            run({3, 0, 1, 2}, block, kind);
            run({100000, 1, 100000, 7}, block, kind);
        }
        run({300000}, 768, kind);
    }

    // a finishing launch in which no thread has a value gives what
    // DeviceReduce::Min gives for an empty input
    toi_min_test::toi_min_test_kernel<<<1, 32>>>(
        nullptr, 0, TOIMinSlot{slot.data(), slot.data() + 1}, out.data());
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
    double a = 0;
    out.copy_to(&a);
    REQUIRE(a == std::numeric_limits<double>::max());
    std::vector<unsigned long long> s;
    slot.copy_to(s);
    REQUIRE(s[0] == TOIMinIdentity);
    REQUIRE(s[1] == 0);
}
