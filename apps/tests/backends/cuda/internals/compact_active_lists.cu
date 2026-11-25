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

// InfoStacklessBVHSimplexTrajectoryFilter compacts its four filter_active
// lists (PP, PE, PT, EE) with compact_active_lists() in one launch instead of
// four DeviceSelect::If calls. This case requires DeviceSelect's outputs and
// counts on lists around the tile size and the length limit.

#include <app/app.h>
#include <collision_detection/filters/compact_active_lists.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
template <typename Vec>
struct CompactActiveListsTestKept
{
    CUB_RUNTIME_FUNCTION bool operator()(const Vec& v) const
    {
        return v(0) != -1;
    }
};

// One list: random indices, the first set to -1 with probability `drop`.
template <typename Vec>
struct CompactActiveListsTestList
{
    std::vector<Vec>                host;
    cuda_tool::DeviceBuffer<Vec>    in, out, reference;
    cuda_tool::DeviceBuffer<IndexT> count;

    CompactActiveListsTestList(int n, double drop, std::mt19937_64& rng)
    {
        std::uniform_int_distribution<IndexT> index(0, 1 << 20);
        std::bernoulli_distribution           dropped(drop);
        host.resize(n);
        for(Vec& v : host)
        {
            for(int k = 0; k < Vec::RowsAtCompileTime; ++k)
                v(k) = index(rng);
            if(dropped(rng))
                v(0) = -1;
        }
        in.copy_from(host.data(), host.size());
        // Poisoned outputs: nothing past the kept elements may be written.
        std::vector<Vec> poison(n, Vec::Constant(0x7f7f7f7f));
        out.copy_from(poison.data(), poison.size());
        reference.copy_from(poison.data(), poison.size());
        count.resize(1, -7);
    }

    CompactActiveList list()
    {
        return {reinterpret_cast<const IndexT*>(in.data()),
                reinterpret_cast<IndexT*>(out.data()),
                count.data(),
                static_cast<int>(host.size()),
                Vec::RowsAtCompileTime};
    }

    void check()
    {
        cuda_tool::DeviceBuffer<IndexT> reference_count(1);
        cuda_tool::DeviceSelect().If(in.data(),
                                     reference.data(),
                                     reference_count.data(),
                                     static_cast<int>(host.size()),
                                     CompactActiveListsTestKept<Vec>{});
        std::vector<IndexT> c, rc;
        count.copy_to(c);
        reference_count.copy_to(rc);
        SizeT expected = 0;
        for(const Vec& v : host)
            expected += v(0) != -1 ? 1 : 0;
        REQUIRE(rc[0] == IndexT(expected));
        REQUIRE(c[0] == rc[0]);

        std::vector<Vec> a, b;
        out.copy_to(a);
        reference.copy_to(b);
        REQUIRE(a.size() == host.size());
        // the kept elements as DeviceSelect wrote them, the rest untouched
        REQUIRE(std::memcmp(a.data(), b.data(), sizeof(Vec) * c[0]) == 0);
        for(SizeT i = c[0]; i < a.size(); ++i)
            REQUIRE(a[i] == Vec::Constant(0x7f7f7f7f));
    }
};

void check_compact_active_lists(std::array<int, 4>    sizes,
                                std::array<double, 4> drops,
                                std::uint64_t         seed)
{
    std::mt19937_64                      rng{seed};
    CompactActiveListsTestList<Vector2i> PP(sizes[0], drops[0], rng);
    CompactActiveListsTestList<Vector3i> PE(sizes[1], drops[1], rng);
    CompactActiveListsTestList<Vector4i> PT(sizes[2], drops[2], rng);
    CompactActiveListsTestList<Vector4i> EE(sizes[3], drops[3], rng);
    REQUIRE(compact_active_lists({PP.list(), PE.list(), PT.list(), EE.list()}, nullptr));
    CUDA_TOOL_CHECK(cudaGetLastError());
    PP.check();
    PE.check();
    PT.check();
    EE.check();
}
}  // namespace

TEST_CASE("compact_active_lists matches DeviceSelect on the four filter_active lists",
          "[cuda][collision detection][compact]")
{
    constexpr int M = CompactActiveListMaxSize;
    check_compact_active_lists({0, 0, 0, 0}, {0.5, 0.5, 0.5, 0.5}, 1);
    check_compact_active_lists({1, 511, 512, 513}, {0.0, 0.5, 1.0, 0.3}, 2);
    check_compact_active_lists({1023, 1024, 1025, 0}, {0.9, 0.1, 0.5, 0.5}, 3);
    check_compact_active_lists({9616, 9616, 1936, 7680}, {0.97, 0.9, 0.85, 0.9}, 4);
    check_compact_active_lists({M, M - 1, 2, M}, {0.5, 0.0, 1.0, 0.999}, 5);

    // A list over the limit: nothing launches, the caller takes CUB.
    std::mt19937_64                      rng{6};
    CompactActiveListsTestList<Vector2i> PP(M + 1, 0.5, rng);
    CompactActiveListsTestList<Vector3i> PE(4, 0.5, rng);
    CompactActiveListsTestList<Vector4i> PT(4, 0.5, rng);
    CompactActiveListsTestList<Vector4i> EE(4, 0.5, rng);
    REQUIRE_FALSE(compact_active_lists({PP.list(), PE.list(), PT.list(), EE.list()}, nullptr));
    std::vector<IndexT> c;
    PE.count.copy_to(c);
    REQUIRE(c[0] == -7);
}
