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

// The CCD and active-set filter kernels launch with one-warp blocks
// (cuda_tool::spread_block_dim) instead of the occupancy-maximizing block.
// This case runs the filters' CCD routine both ways on random approaching
// pairs and requires the same bits from every item.

#include <app/app.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/distance.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
// PT: p, t0, t1, t2; EE: a0, a1, b0, b1. Four positions and four
// displacements per item, as compute_feasible_step_{PT,EE}_kernel read them.
__global__ void ccd_launch_shape_kernel(
    const Vector3* xs, const Vector3* dxs, bool point_triangle, Float* tois, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const Vector3* x   = xs + 4 * i;
    const Vector3* dx  = dxs + 4 * i;
    Float          toi = 1.0;
    bool           hit;
    if(point_triangle)
        hit = distance::point_triangle_ccd(
            x[0], x[1], x[2], x[3], dx[0], dx[1], dx[2], dx[3], 0.2, 0.0, 1000, toi);
    else
        hit = distance::edge_edge_ccd(
            x[0], x[1], x[2], x[3], dx[0], dx[1], dx[2], dx[3], 0.2, 0.0, 1000, toi);
    tois[i] = hit ? toi : Float(1.0);
}

void check_launch_shape(bool point_triangle, int n, std::uint64_t seed)
{
    std::mt19937_64                       rng{seed};
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    auto rand3 = [&](Float s) { return Vector3{u(rng), u(rng), u(rng)} * s; };

    // A 10 cm primitive pair 1 cm apart near (100, 100, 100): the point (or
    // the second edge) moves down by 0.5-1.5 times the gap, the others jitter.
    std::vector<Vector3> xs, dxs;
    for(int i = 0; i < n; ++i)
    {
        Vector3 c = Vector3::Constant(100.0) + rand3(1.0);
        Vector3 x[4];
        if(point_triangle)
        {
            x[0] = c + Vector3{0.02 * u(rng), 0.02 * u(rng), 0.01};
            x[1] = c + Vector3{-0.05, -0.05, 0.0} + rand3(0.002);
            x[2] = c + Vector3{0.05, -0.05, 0.0} + rand3(0.002);
            x[3] = c + Vector3{0.0, 0.05, 0.0} + rand3(0.002);
        }
        else
        {
            x[0] = c + Vector3{-0.05, 0.0, 0.0} + rand3(0.002);
            x[1] = c + Vector3{0.05, 0.0, 0.0} + rand3(0.002);
            x[2] = c + Vector3{0.0, -0.05, 0.01} + rand3(0.002);
            x[3] = c + Vector3{0.0, 0.05, 0.01} + rand3(0.002);
        }
        Vector3 d = Vector3{0.0, 0.0, -0.01 * (1.0 + 0.5 * u(rng))};
        for(int k = 0; k < 4; ++k)
        {
            bool moves = point_triangle ? k == 0 : k >= 2;
            xs.push_back(x[k]);
            dxs.push_back(moves ? Vector3{d + rand3(0.0005)} : rand3(0.0005));
        }
    }

    cuda_tool::DeviceBuffer<Vector3> d_xs, d_dxs;
    cuda_tool::DeviceBuffer<Float>   occupancy, spread;
    d_xs.copy_from(xs.data(), xs.size());
    d_dxs.copy_from(dxs.data(), dxs.size());
    // NaN in every slot, so an item a launch shape misses cannot pass.
    std::vector<Float> poison(n, std::numeric_limits<Float>::quiet_NaN());
    occupancy.copy_from(poison.data(), poison.size());
    spread.copy_from(poison.data(), poison.size());

    auto k = ccd_launch_shape_kernel;
    k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
        d_xs.data(), d_dxs.data(), point_triangle, occupancy.data(), n);
    CUDA_TOOL_CHECK(cudaGetLastError());
    k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim>>>(
        d_xs.data(), d_dxs.data(), point_triangle, spread.data(), n);
    CUDA_TOOL_CHECK(cudaGetLastError());

    std::vector<Float> a, b;
    occupancy.copy_to(a);
    spread.copy_to(b);
    REQUIRE(a.size() == SizeT(n));
    REQUIRE(b.size() == SizeT(n));
    REQUIRE(std::memcmp(a.data(), b.data(), sizeof(Float) * n) == 0);

    SizeT hits = 0, finite = 0;
    for(Float t : b)
    {
        finite += std::isfinite(t) ? 1 : 0;
        hits += t < 1.0 ? 1 : 0;
    }
    REQUIRE(finite == SizeT(n));
    if(n >= 1000)  // both outcomes, so the iterative path runs too
    {
        REQUIRE(hits > 0);
        REQUIRE(hits < SizeT(n));
    }
}
}  // namespace

TEST_CASE("CCD filters give the same bits with one-warp blocks", "[cuda][collision detection][launch]")
{
    REQUIRE(cuda_tool::spread_block_dim == 32);
    for(int n : {1, 31, 32, 33, 4095, 4096, 4097})
    {
        int grid = cuda_tool::spread_grid_dim(n);
        REQUIRE(SizeT(grid) * cuda_tool::spread_block_dim >= SizeT(n));
        REQUIRE(SizeT(grid - 1) * cuda_tool::spread_block_dim < SizeT(n));
    }
    int seed = 0;
    for(int n : {1, 33, 1000, 5000})
    {
        check_launch_shape(true, n, 101 + seed);
        check_launch_shape(false, n, 202 + seed);
        ++seed;
    }
}
