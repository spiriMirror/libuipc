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

// The small read-backs of the Newton loop (the TOI, the active-set counts,
// the trial energy total, the judge counts, the BCOO counts, the axis
// maximum displacement) are stored by their producers into a
// cuda_tool::ReadbackBuffer: on the mapped path straight into mapped
// page-locked host memory, on the copy path into device memory followed by
// a device-to-host copy. Every case runs each producer the way its site
// does, once on each path, and requires the same bytes on the host: a kernel
// store, the in-kernel TOI minimum and the half-plane filter's n = 0 path
// (fill kernel plus DeviceReduce::Min), the simplex filter's "no candidate"
// store (store_value), the one-launch compaction with empty lists and its
// DeviceSelect::If fallback, the energy sum with and without reporters, and
// a CUB maximum.

#include <app/app.h>
#include <collision_detection/filters/compact_active_lists.h>
#include <collision_detection/filters/toi_min.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace readback_buffer_test
{
struct Counts
{
    int upper_count;
    int unique_count;
};

__global__ void store_counts_kernel(cuda_tool::Dense<Counts> device_counts,
                                    cuda_tool::Dense<Counts> readback,
                                    int                      upper,
                                    int                      unique)
{
    device_counts->upper_count  = upper;
    device_counts->unique_count = unique;
    *readback                   = *device_counts;
}

__global__ void store_values_kernel(cuda_tool::BufferView<IndexT> out, IndexT base)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < static_cast<int>(out.size()))
        out(i) = base + 3 * i;
}

__global__ void toi_kernel(const double* v, int n, TOIMinSlot slot, double* out)
{
    int                i   = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned long long key = TOIMinIdentity;
    if(i < n)
        key = toi_min_key(v[i]);
    toi_min_block(key, slot, out);
}

struct Kept
{
    template <typename Vec>
    CUB_RUNTIME_FUNCTION bool operator()(const Vec& v) const
    {
        return v(0) != -1;
    }
};

template <typename T>
std::vector<unsigned char> bytes(const T* p, SizeT n)
{
    std::vector<unsigned char> b(n * sizeof(T));
    std::memcpy(b.data(), p, b.size());
    return b;
}

// Runs `produce(device_data, stream)` into a fresh buffer of `n` elements on
// the given path and returns the host bytes after enqueue_copy() and the
// stream synchronization, as the sites do.
template <typename T, typename Produce>
std::vector<unsigned char> read_back(cuda_tool::ReadbackPath path, SizeT n, Produce&& produce)
{
    cuda_tool::ReadbackBuffer<T> rb{path};
    rb.resize_discard(n);
    if(path == cuda_tool::ReadbackPath::Copy)
        REQUIRE_FALSE(rb.mapped());
    else
        REQUIRE(rb.mapped() == cuda_tool::mapped_readbacks());
    // poison the host side: every byte compared must be written
    std::memset(rb.data(), 0x5a, n * sizeof(T));
    cudaStream_t s = cuda_tool::default_stream();
    produce(rb.device_data(), s);
    rb.enqueue_copy(s);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    return bytes(rb.data(), n);
}

template <typename T, typename Produce>
std::vector<unsigned char> both_paths(SizeT n, Produce&& produce)
{
    auto mapped = read_back<T>(cuda_tool::ReadbackPath::Auto, n, produce);
    auto copied = read_back<T>(cuda_tool::ReadbackPath::Copy, n, produce);
    REQUIRE(mapped == copied);
    return mapped;
}
}  // namespace readback_buffer_test

using namespace readback_buffer_test;

TEST_CASE("ReadbackBuffer: mapped and copy paths read back the producer's bytes",
          "[cuda][cuda_tool][readback]")
{
    // The test GPUs map host memory; the copy path is covered by
    // ReadbackPath::Copy here.
    INFO("mapped_readbacks() = " << cuda_tool::mapped_readbacks());

    SECTION("kernel stores (judge counts, compaction counts)")
    {
        for(SizeT n : {SizeT{1}, SizeT{4}, SizeT{8}, SizeT{1000}})
        {
            auto b =
                both_paths<IndexT>(n,
                                   [&](IndexT* d, cudaStream_t s)
                                   {
                                       int threads = 128;
                                       int blocks = int((n + threads - 1) / threads);
                                       store_values_kernel<<<blocks, threads, 0, s>>>(
                                           cuda_tool::BufferView<IndexT>{d, n}, 7);
                                   });
            std::vector<IndexT> v(n);
            std::memcpy(v.data(), b.data(), b.size());
            for(SizeT i = 0; i < n; ++i)
                REQUIRE(v[i] == IndexT(7 + 3 * i));
        }
    }

    SECTION("a struct store beside the device copy (BCOO k3)")
    {
        cuda_tool::DeviceVar<Counts> device_counts;
        auto                         b = both_paths<Counts>(
            1,
            [&](Counts* d, cudaStream_t s)
            {
                store_counts_kernel<<<1, 1, 0, s>>>(
                    device_counts.viewer(), cuda_tool::Dense<Counts>{d}, 12345, 678);
            });
        Counts c;
        std::memcpy(&c, b.data(), sizeof(Counts));
        REQUIRE(c.upper_count == 12345);
        REQUIRE(c.unique_count == 678);
        Counts on_device = device_counts;
        REQUIRE(std::memcmp(&on_device, &c, sizeof(Counts)) == 0);
    }

    SECTION("a host copy into the host elements (DeviceSelect fallback)")
    {
        std::vector<IndexT>             host = {3, 1, 4, 1};
        cuda_tool::DeviceBuffer<IndexT> src;
        src.copy_from(host.data(), host.size());
        for(auto path : {cuda_tool::ReadbackPath::Auto, cuda_tool::ReadbackPath::Copy})
        {
            cuda_tool::ReadbackBuffer<IndexT> rb{path};
            rb.resize_discard(4);
            std::memset(rb.data(), 0x5a, 4 * sizeof(IndexT));
            rb.copy_from(0, src.cview(), cuda_tool::default_stream());
            CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
            REQUIRE(std::memcmp(rb.data(), host.data(), 4 * sizeof(IndexT)) == 0);
        }
    }

    SECTION("growth keeps the path and the device address follows")
    {
        cuda_tool::ReadbackBuffer<double> rb;
        rb.resize_discard(1);
        auto* first = rb.device_data();
        rb.resize_discard(1);
        REQUIRE(rb.device_data() == first);
        rb.resize_discard(64);
        REQUIRE(rb.size() == 64);
        REQUIRE(rb.mapped() == cuda_tool::mapped_readbacks());
        cuda_tool::BufferLaunch().fill(
            cuda_tool::BufferView<double>{rb.device_data(), 64}, 2.5);
        rb.enqueue_copy();
        CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
        for(SizeT i = 0; i < 64; ++i)
            REQUIRE(rb[i] == 2.5);
    }
}

TEST_CASE("ReadbackBuffer: TOI minimum, half-plane n = 0 and simplex no-candidate paths",
          "[cuda][cuda_tool][readback][toi min]")
{
    std::mt19937_64                        rng{41};
    std::uniform_real_distribution<double> unit{0.0, 1.0};

    cuda_tool::DeviceBuffer<unsigned long long> slot;
    const unsigned long long                    init[2] = {TOIMinIdentity, 0};
    slot.copy_from(init, 2);

    for(int n : {1, 31, 32, 33, 1000, 5000})
    {
        std::vector<double> v(n);
        for(auto& x : v)
            x = unit(rng) < 0.3 ? 1.1 : 0.05 + unit(rng);
        cuda_tool::DeviceBuffer<double> dv;
        dv.copy_from(v.data(), v.size());
        auto b = both_paths<double>(
            1,
            [&](double* out, cudaStream_t s)
            {
                toi_kernel<<<(n + 31) / 32, 32, 0, s>>>(
                    dv.data(), n, TOIMinSlot{slot.data(), slot.data() + 1}, out);
            });
        double m;
        std::memcpy(&m, b.data(), sizeof(double));
        REQUIRE(m == *std::min_element(v.begin(), v.end()));
    }
    // the slot is reset after every use
    std::vector<unsigned long long> after;
    slot.copy_to(after);
    REQUIRE(after[0] == TOIMinIdentity);
    REQUIRE(after[1] == 0);

    // EasyVertexHalfPlaneTrajectoryFilter without surface vertices: a fill
    // kernel and DeviceReduce::Min over nothing, both into the TOI slot
    cuda_tool::DeviceBuffer<double> none(1);
    both_paths<double>(1,
                       [&](double* out, cudaStream_t s)
                       {
                           cuda_tool::BufferLaunch(s).fill(
                               cuda_tool::BufferView<double>{out, 1}, double(1.1f));
                           cuda_tool::DeviceReduce(s).Min(none.data(), out, 0);
                       });

    // InfoStacklessBVHSimplexTrajectoryFilter without candidates
    // (last < 0): the "no hit" TOI stored from the host
    auto   b = both_paths<double>(1,
                                [&](double* out, cudaStream_t s)
                                {
                                    const double large_enough_toi = 1.1;
                                    cuda_tool::store_value(out, large_enough_toi, s);
                                });
    double m;
    std::memcpy(&m, b.data(), sizeof(double));
    REQUIRE(m == 1.1);
}

TEST_CASE("ReadbackBuffer: compaction counts, empty lists and the DeviceSelect fallback",
          "[cuda][cuda_tool][readback][compact active lists]")
{
    std::mt19937_64                       rng{43};
    std::uniform_int_distribution<IndexT> index(0, 1 << 20);
    std::bernoulli_distribution           dropped(0.4);

    auto make = [&](int n)
    {
        std::vector<Vector4i> h(n);
        for(auto& x : h)
        {
            for(int k = 0; k < 4; ++k)
                x(k) = index(rng);
            if(dropped(rng))
                x(0) = -1;
        }
        return h;
    };

    // empty lists, a tile, a partial tile and the length limit
    for(std::array<int, 4> sizes : {std::array<int, 4>{0, 0, 0, 0},
                                    std::array<int, 4>{0, 17, 0, 300},
                                    std::array<int, 4>{256, 0, 1, CompactActiveListMaxSize}})
    {
        std::array<std::vector<Vector4i>, 4> h;
        std::array<cuda_tool::DeviceBuffer<Vector4i>, 4> in, out_mapped, out_copied;
        for(int l = 0; l < 4; ++l)
        {
            h[l] = make(sizes[l]);
            in[l].copy_from(h[l].data(), h[l].size());
            out_mapped[l].resize(h[l].size());
            out_copied[l].resize(h[l].size());
        }
        using Outs = std::array<cuda_tool::DeviceBuffer<Vector4i>, 4>;
        auto run   = [&in, &sizes](Outs* outs)
        {
            return [&in, &sizes, outs](IndexT* counts, cudaStream_t s)
            {
                std::array<CompactActiveList, 4> lists;
                for(int l = 0; l < 4; ++l)
                    lists[l] =
                        CompactActiveList{reinterpret_cast<const IndexT*>(in[l].data()),
                                          reinterpret_cast<IndexT*>((*outs)[l].data()),
                                          counts + l,
                                          sizes[l],
                                          4};
                REQUIRE(compact_active_lists(lists, s));
            };
        };
        auto mapped =
            read_back<IndexT>(cuda_tool::ReadbackPath::Auto, 4, run(&out_mapped));
        auto copied =
            read_back<IndexT>(cuda_tool::ReadbackPath::Copy, 4, run(&out_copied));
        REQUIRE(mapped == copied);
        std::array<IndexT, 4> counts;
        std::memcpy(counts.data(), mapped.data(), sizeof(counts));
        for(int l = 0; l < 4; ++l)
        {
            IndexT kept = 0;
            for(auto& x : h[l])
                kept += x(0) != -1;
            REQUIRE(counts[l] == kept);
            std::vector<Vector4i> a, c;
            out_mapped[l].copy_to(a);
            out_copied[l].copy_to(c);
            REQUIRE(std::memcmp(a.data(), c.data(), sizeof(Vector4i) * kept) == 0);
        }
    }

    // A list longer than the limit: compact_active_lists() refuses and the
    // filter runs DeviceSelect::If into device counts, then copies them into
    // the read-back buffer's host elements.
    {
        const int                         n = CompactActiveListMaxSize + 1;
        auto                              h = make(n);
        cuda_tool::DeviceBuffer<Vector4i> in, out;
        in.copy_from(h.data(), h.size());
        out.resize(n);
        cuda_tool::DeviceBuffer<IndexT> device_counts(4);
        for(auto path : {cuda_tool::ReadbackPath::Auto, cuda_tool::ReadbackPath::Copy})
        {
            cuda_tool::ReadbackBuffer<IndexT> rb{path};
            rb.resize_discard(4);
            std::memset(rb.data(), 0x5a, 4 * sizeof(IndexT));
            std::array<CompactActiveList, 4> lists;
            for(int l = 0; l < 4; ++l)
                lists[l] = CompactActiveList{reinterpret_cast<const IndexT*>(in.data()),
                                             reinterpret_cast<IndexT*>(out.data()),
                                             rb.device_data() + l,
                                             n,
                                             4};
            REQUIRE_FALSE(compact_active_lists(lists, nullptr));
            for(int l = 0; l < 4; ++l)
                cuda_tool::DeviceSelect().If(
                    in.data(), out.data(), device_counts.data() + l, n, Kept{});
            rb.copy_from(0, device_counts.cview(), nullptr);
            CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
            IndexT kept = 0;
            for(auto& x : h)
                kept += x(0) != -1;
            for(int l = 0; l < 4; ++l)
                REQUIRE(rb[l] == kept);
        }
    }
}

TEST_CASE("ReadbackBuffer: energy total with and without reporters, axis maximum",
          "[cuda][cuda_tool][readback][line search]")
{
    std::mt19937_64                        rng{47};
    std::uniform_real_distribution<double> e(-1e3, 1e5);

    for(int reporters : {1, 2, 3, 7})
    {
        std::vector<double> v(reporters);
        for(auto& x : v)
            x = e(rng);
        cuda_tool::DeviceBuffer<double> dv;
        dv.copy_from(v.data(), v.size());
        auto b = both_paths<double>(
            1,
            [&](double* total, cudaStream_t s)
            { cuda_tool::DeviceReduce(s).Sum(dv.data(), total, reporters); });
        // the same reduction into device memory and a plain copy
        cuda_tool::DeviceVar<double> reference;
        cuda_tool::DeviceReduce().Sum(dv.data(), reference.data(), reporters);
        double r = reference;
        REQUIRE(std::memcmp(&r, b.data(), sizeof(double)) == 0);
    }

    // no reporter: the total is stored from the host
    auto   b = both_paths<double>(1,
                                [&](double* total, cudaStream_t s)
                                { cuda_tool::store_value(total, double{0}, s); });
    double zero;
    std::memcpy(&zero, b.data(), sizeof(double));
    REQUIRE(zero == 0.0);
    REQUIRE_FALSE(std::signbit(zero));

    // the axis maximum displacement is a CUB maximum
    std::vector<double> d(3 * 5000);
    for(auto& x : d)
        x = e(rng);
    cuda_tool::DeviceBuffer<double> dd;
    dd.copy_from(d.data(), d.size());
    auto   m = both_paths<double>(1,
                                [&](double* out, cudaStream_t s)
                                {
                                    cuda_tool::DeviceReduce(s).Max(
                                        dd.data(), out, static_cast<int>(d.size()));
                                });
    double mx;
    std::memcpy(&mx, m.data(), sizeof(double));
    REQUIRE(mx == *std::max_element(d.begin(), d.end()));
}
