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

// InfoStacklessBVHSimplexTrajectoryFilter writes a detect's swept boxes, cover
// gathers, FP32 inputs and containment flags with launch_trajectory_prefix()
// in two launches. This case runs the separate kernels it replaces (copied
// below from the filter) on random primitives, splits and grown boxes, NaN
// and infinite coordinates included, and requires the same bytes.

#include <app/app.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>
#include <utils/step_displacement.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
// detect_k1_kernel and detect_k2_kernel of the filter (the same code).
__global__ void trajectory_prefix_test_point_kernel(
    cuda_tool::CBufferView<IndexT>  Vs,
    cuda_tool::CBufferView<Vector3> Ps,
    cuda_tool::CBufferView<Vector3> dxs,
    cuda_tool::BufferView<AABB>     aabbs,
    cuda_tool::CBufferView<IndexT>  v2bs,
    cuda_tool::CBufferView<IndexT>  contact_ids,
    cuda_tool::BufferView<IndexT>   bids,
    cuda_tool::BufferView<IndexT>   cids,
    cuda_tool::CBufferView<Float>   thicknesses,
    cuda_tool::CBufferView<Float>   d_hats,
    Float                           alpha,
    int                             n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    auto vI = Vs(i);

    Float thickness       = thicknesses(vI);
    Float d_hat_expansion = point_dcd_expansion(d_hats(vI));

    const auto& pos   = Ps(vI);
    Vector3     pos_t = pos + dxs(vI) * alpha;

    AABB aabb;
    aabb.extend(pos.cast<float>()).extend(pos_t.cast<float>());

    float expand = d_hat_expansion + thickness;

    aabb.min().array() -= expand;
    aabb.max().array() += expand;
    aabbs(i) = aabb;
    bids(i)  = v2bs(vI);
    cids(i)  = contact_ids(vI);
}

// detect_k3_kernel of the filter.
__global__ void trajectory_prefix_test_edge_kernel(
    cuda_tool::CBufferView<Vector2i> Es,
    cuda_tool::CBufferView<Vector3>  Ps,
    cuda_tool::BufferView<AABB>      aabbs,
    cuda_tool::CBufferView<IndexT>   v2bs,
    cuda_tool::CBufferView<IndexT>   contact_ids,
    cuda_tool::BufferView<IndexT>    bids,
    cuda_tool::BufferView<IndexT>    cids,
    cuda_tool::CBufferView<Vector3>  dxs,
    cuda_tool::CBufferView<Float>    thicknesses,
    cuda_tool::CBufferView<Float>    d_hats,
    Float                            alpha,
    int                              n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    auto eI = Es(i);

    Float thickness = edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));
    Float d_hat_expansion = edge_dcd_expansion(d_hats(eI[0]), d_hats(eI[1]));

    const auto& pos0   = Ps(eI[0]);
    const auto& pos1   = Ps(eI[1]);
    Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
    Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

    AABB aabb;

    aabb.extend(pos0.cast<float>())
        .extend(pos1.cast<float>())
        .extend(pos0_t.cast<float>())
        .extend(pos1_t.cast<float>());

    float expand = d_hat_expansion + thickness;

    aabb.min().array() -= expand;
    aabb.max().array() += expand;
    aabbs(i) = aabb;
    bids(i)  = v2bs(eI[0]);
    cids(i)  = contact_ids(eI[0]);
}

// detect_k4_kernel of the filter.
__global__ void trajectory_prefix_test_triangle_kernel(
    cuda_tool::CBufferView<Vector3i> Fs,
    cuda_tool::CBufferView<Vector3>  Ps,
    cuda_tool::BufferView<AABB>      aabbs,
    cuda_tool::CBufferView<IndexT>   v2bs,
    cuda_tool::CBufferView<IndexT>   contact_ids,
    cuda_tool::BufferView<IndexT>    bids,
    cuda_tool::BufferView<IndexT>    cids,
    cuda_tool::CBufferView<Vector3>  dxs,
    cuda_tool::CBufferView<Float>    thicknesses,
    cuda_tool::CBufferView<Float>    d_hats,
    Float                            alpha,
    int                              n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    auto fI = Fs(i);

    Float thickness =
        triangle_thickness(thicknesses(fI[0]), thicknesses(fI[1]), thicknesses(fI[2]));
    Float d_hat_expansion =
        triangle_dcd_expansion(d_hats(fI[0]), d_hats(fI[1]), d_hats(fI[2]));

    const auto& pos0   = Ps(fI[0]);
    const auto& pos1   = Ps(fI[1]);
    const auto& pos2   = Ps(fI[2]);
    Vector3     pos0_t = pos0 + dxs(fI[0]) * alpha;
    Vector3     pos1_t = pos1 + dxs(fI[1]) * alpha;
    Vector3     pos2_t = pos2 + dxs(fI[2]) * alpha;

    AABB aabb;

    aabb.extend(pos0.cast<float>())
        .extend(pos1.cast<float>())
        .extend(pos2.cast<float>())
        .extend(pos0_t.cast<float>())
        .extend(pos1_t.cast<float>())
        .extend(pos2_t.cast<float>());

    float expand = d_hat_expansion + thickness;

    aabb.min().array() -= expand;
    aabb.max().array() += expand;
    aabbs(i) = aabb;
    bids(i)  = v2bs(fI[0]);
    cids(i)  = contact_ids(fI[0]);
}

// f32_inputs_kernel of the filter.
__global__ void trajectory_prefix_test_f32_kernel(cuda_tool::CBufferView<Vector3> Ps,
                                                  cuda_tool::CBufferView<Vector3> dxs,
                                                  cuda_tool::CBufferView<Float> d_hats,
                                                  cuda_tool::CBufferView<Float> thicknesses,
                                                  Float alpha,
                                                  cuda_tool::BufferView<float4> positions,
                                                  cuda_tool::BufferView<float4> steps,
                                                  int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const Vector3 x = Ps(i);
    const Vector3 d = step_displacement(alpha, dxs(i));
    positions(i)    = make_float4(static_cast<float>(x.x()),
                               static_cast<float>(x.y()),
                               static_cast<float>(x.z()),
                               static_cast<float>(d_hats(i)));
    steps(i)        = make_float4(static_cast<float>(d.x()),
                           static_cast<float>(d.y()),
                           static_cast<float>(d.z()),
                           static_cast<float>(thicknesses(i)));
}

// cover_gather_kernel of the filter.
__global__ void trajectory_prefix_test_gather_kernel(cuda_tool::CBufferView<IndexT> ids,
                                                     cuda_tool::CBufferView<AABB> aabbs,
                                                     cuda_tool::CBufferView<IndexT> bids,
                                                     cuda_tool::CBufferView<IndexT> cids,
                                                     cuda_tool::BufferView<AABB> out_aabbs,
                                                     cuda_tool::BufferView<IndexT> out_bids,
                                                     cuda_tool::BufferView<IndexT> out_cids,
                                                     int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    IndexT id    = ids(i);
    out_aabbs(i) = aabbs(id);
    out_bids(i)  = bids(id);
    out_cids(i)  = cids(id);
}

// contained_kernel of the filter.
__global__ void trajectory_prefix_test_contained_kernel(cuda_tool::CBufferView<AABB> aabbs,
                                                        cuda_tool::CBufferView<AABB> grown,
                                                        cuda_tool::CBufferView<IndexT> is_query,
                                                        float cover_shrink,
                                                        float other_shrink,
                                                        cuda_tool::BufferView<IndexT> escaped,
                                                        int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= n)
        return;
    const AABB& box    = aabbs(i);
    const AABB& g      = grown(i);
    bool        inside = (g.min().array() <= box.min().array()).all()
                  && (box.max().array() <= g.max().array()).all();
    if(!inside)
        escaped(0) = 1;
    if(is_query.size() == 0)
        return;
    const bool  cover = is_query(i) != 0;
    const float s     = cover ? cover_shrink : other_shrink;
    bool        lower = ((g.min().array() + s) <= box.min().array()).all()
                 && (box.max().array() <= (g.max().array() - s)).all();
    if(!lower)
        escaped(cover ? 1 : 2) = 1;
}

constexpr int TestBlock = 256;

int grid(int n)
{
    return (n + TestBlock - 1) / TestBlock;
}

template <typename T>
std::vector<T> download(const cuda_tool::DeviceBuffer<T>& b)
{
    std::vector<T> h;
    b.copy_to(h);
    return h;
}

template <typename T>
bool same_bytes(const cuda_tool::DeviceBuffer<T>& a, const cuda_tool::DeviceBuffer<T>& b)
{
    auto x = download(a);
    auto y = download(b);
    return x.size() == y.size()
           && (x.empty() || std::memcmp(x.data(), y.data(), sizeof(T) * x.size()) == 0);
}

template <typename T>
void poison(cuda_tool::DeviceBuffer<T>& b, SizeT n)
{
    std::vector<T> h(n);
    std::memset(static_cast<void*>(h.data()), 0x5a, sizeof(T) * n);
    b.resize(n);
    b.copy_from(h.data(), n);
}

// One family's outputs, written by the reference kernels or by the prefix.
struct TrajectoryPrefixTestOut
{
    cuda_tool::DeviceBuffer<AABB>   aabbs, gathered_aabbs;
    cuda_tool::DeviceBuffer<IndexT> bids, cids, gathered_bids, gathered_cids;

    void reset(SizeT n, SizeT gathered)
    {
        poison(aabbs, n);
        poison(bids, n);
        poison(cids, n);
        poison(gathered_aabbs, gathered);
        poison(gathered_bids, gathered);
        poison(gathered_cids, gathered);
    }
};

void check_trajectory_prefix(
    int nv, int ncodim, int npoints, int nedges, int ntris, Float alpha, bool contained, std::uint64_t seed)
{
    std::mt19937_64                       rng{seed};
    std::uniform_real_distribution<Float> u(-1.0, 1.0);
    std::uniform_int_distribution<int>    vertex(0, std::max(nv - 1, 0));
    std::bernoulli_distribution           rare(0.003);

    // Vertices around a scene-sized offset, displacements of mixed scales,
    // a few NaN and infinite coordinates. d_hat and thickness are uniform
    // within each of four vertex groups (vertex index mod 4; the box kernels
    // assert them equal over a primitive), and each primitive takes its
    // vertices from one group.
    constexpr int Groups = 4;
    Float         group_d_hat[Groups], group_thickness[Groups];
    for(int g = 0; g < Groups; ++g)
    {
        group_d_hat[g]     = 1e-3 * (2.0 + u(rng));
        group_thickness[g] = g == 0 ? 0.0 : 1e-4 * (1.0 + u(rng));
    }
    std::vector<Vector3> Ps(nv), dxs(nv);
    std::vector<Float>   d_hats(nv), thicknesses(nv);
    std::vector<IndexT>  v2bs(nv), contact_ids(nv);
    for(int i = 0; i < nv; ++i)
    {
        Ps[i] = Vector3{u(rng), u(rng), u(rng)} * 3.0 + Vector3{100.0, -2.0, 0.5};
        dxs[i] = Vector3{u(rng), u(rng), u(rng)}
                 * std::pow(10.0, -4.0 + 3.0 * (u(rng) + 1.0));
        if(rare(rng))
            Ps[i][i % 3] = std::numeric_limits<Float>::quiet_NaN();
        if(rare(rng))
            dxs[i][(i + 1) % 3] = std::numeric_limits<Float>::infinity();
        d_hats[i]      = group_d_hat[i % Groups];
        thicknesses[i] = group_thickness[i % Groups];
        v2bs[i]        = (i % 11 == 0) ? -1 : i / 37;
        contact_ids[i] = (i % 13 == 0) ? 9 : (i / 50) % 6;
    }
    // n primitives of k vertices, each from one group
    auto pick = [&](int n, int k)
    {
        std::vector<IndexT> v(static_cast<SizeT>(n) * k);
        for(int i = 0; i < n; ++i)
        {
            int g = vertex(rng) % Groups;
            for(int j = 0; j < k; ++j)
            {
                int x                            = vertex(rng);
                x                                = x - x % Groups + g;
                v[static_cast<SizeT>(i) * k + j] = x < nv ? x : g;
            }
        }
        return v;
    };
    std::vector<IndexT>   codimVs = pick(ncodim, 1), Vs = pick(npoints, 1);
    std::vector<IndexT>   e = pick(nedges, 2), t = pick(ntris, 3);
    std::vector<Vector2i> Es(nedges);
    std::vector<Vector3i> Fs(ntris);
    for(int i = 0; i < nedges; ++i)
        Es[i] = {e[2 * i], e[2 * i + 1]};
    for(int i = 0; i < ntris; ++i)
        Fs[i] = {t[3 * i], t[3 * i + 1], t[3 * i + 2]};

    cuda_tool::DeviceBuffer<Vector3>  d_Ps, d_dxs;
    cuda_tool::DeviceBuffer<Float>    d_dh, d_th;
    cuda_tool::DeviceBuffer<IndexT>   d_v2b, d_cid, d_codim, d_Vs;
    cuda_tool::DeviceBuffer<Vector2i> d_Es;
    cuda_tool::DeviceBuffer<Vector3i> d_Fs;
    d_Ps.copy_from(Ps.data(), nv);
    d_dxs.copy_from(dxs.data(), nv);
    d_dh.copy_from(d_hats.data(), nv);
    d_th.copy_from(thicknesses.data(), nv);
    d_v2b.copy_from(v2bs.data(), nv);
    d_cid.copy_from(contact_ids.data(), nv);
    d_codim.copy_from(codimVs.data(), ncodim);
    d_Vs.copy_from(Vs.data(), npoints);
    d_Es.copy_from(Es.data(), nedges);
    d_Fs.copy_from(Fs.data(), ntris);

    // Families: codim points, points, edges, triangles. Points, edges and
    // triangles carry a cover split: is_query per primitive, ids with S first
    // (then the rest in reverse, as DevicePartition writes it), a gathered
    // range (points: all, the others: S) and its inverse.
    const int                       n[4] = {ncodim, npoints, nedges, ntris};
    std::vector<IndexT>             is_query[4], ids[4], slots[4];
    SizeT                           gathered[4] = {0, 0, 0, 0};
    cuda_tool::DeviceBuffer<IndexT> d_is_query[4], d_ids[4], d_slots[4];
    std::bernoulli_distribution     in_cover(0.3);
    for(int f = 1; f < 4; ++f)
    {
        is_query[f].resize(n[f]);
        std::vector<IndexT> rest;
        for(int i = 0; i < n[f]; ++i)
        {
            is_query[f][i] = in_cover(rng) ? 1 : 0;
            (is_query[f][i] ? ids[f] : rest).push_back(i);
        }
        SizeT count = ids[f].size();
        ids[f].insert(ids[f].end(), rest.rbegin(), rest.rend());
        gathered[f] = f == 1 ? static_cast<SizeT>(n[f]) : count;
        slots[f].assign(n[f], -1);
        for(SizeT j = 0; j < gathered[f]; ++j)
            slots[f][ids[f][j]] = static_cast<IndexT>(j);
        d_is_query[f].copy_from(is_query[f].data(), n[f]);
        d_ids[f].copy_from(ids[f].data(), n[f]);
        d_slots[f].copy_from(slots[f].data(), n[f]);
    }

    // Reference: the separate kernels.
    TrajectoryPrefixTestOut ref[4], out[4];
    for(int f = 0; f < 4; ++f)
    {
        ref[f].reset(n[f], gathered[f]);
        out[f].reset(n[f], gathered[f]);
    }
    if(ncodim > 0)
        trajectory_prefix_test_point_kernel<<<grid(ncodim), TestBlock>>>(
            d_codim.view(),
            d_Ps.view(),
            d_dxs.view(),
            ref[0].aabbs.view(),
            d_v2b.view(),
            d_cid.view(),
            ref[0].bids.view(),
            ref[0].cids.view(),
            d_th.view(),
            d_dh.view(),
            alpha,
            ncodim);
    if(npoints > 0)
        trajectory_prefix_test_point_kernel<<<grid(npoints), TestBlock>>>(
            d_Vs.view(),
            d_Ps.view(),
            d_dxs.view(),
            ref[1].aabbs.view(),
            d_v2b.view(),
            d_cid.view(),
            ref[1].bids.view(),
            ref[1].cids.view(),
            d_th.view(),
            d_dh.view(),
            alpha,
            npoints);
    if(nedges > 0)
        trajectory_prefix_test_edge_kernel<<<grid(nedges), TestBlock>>>(
            d_Es.view(),
            d_Ps.view(),
            ref[2].aabbs.view(),
            d_v2b.view(),
            d_cid.view(),
            ref[2].bids.view(),
            ref[2].cids.view(),
            d_dxs.view(),
            d_th.view(),
            d_dh.view(),
            alpha,
            nedges);
    if(ntris > 0)
        trajectory_prefix_test_triangle_kernel<<<grid(ntris), TestBlock>>>(
            d_Fs.view(),
            d_Ps.view(),
            ref[3].aabbs.view(),
            d_v2b.view(),
            d_cid.view(),
            ref[3].bids.view(),
            ref[3].cids.view(),
            d_dxs.view(),
            d_th.view(),
            d_dh.view(),
            alpha,
            ntris);
    for(int f = 1; f < 4; ++f)
    {
        int m = static_cast<int>(gathered[f]);
        if(m > 0)
            trajectory_prefix_test_gather_kernel<<<grid(m), TestBlock>>>(
                d_ids[f].view(0, m),
                ref[f].aabbs.view(),
                ref[f].bids.view(),
                ref[f].cids.view(),
                ref[f].gathered_aabbs.view(),
                ref[f].gathered_bids.view(),
                ref[f].gathered_cids.view(),
                m);
    }
    cuda_tool::DeviceBuffer<float4> ref_pos, ref_steps, pos, steps, ends;
    poison(ref_pos, nv);
    poison(ref_steps, nv);
    poison(pos, nv);
    poison(steps, nv);
    poison(ends, nv);
    if(nv > 0)
        trajectory_prefix_test_f32_kernel<<<grid(nv), TestBlock>>>(d_Ps.view(),
                                                                   d_dxs.view(),
                                                                   d_dh.view(),
                                                                   d_th.view(),
                                                                   alpha,
                                                                   ref_pos.view(),
                                                                   ref_steps.view(),
                                                                   nv);

    // Grown copies: the reference box grown by a random skin, some shrunk
    // (escapes), the shrinks of the lower skin levels in between.
    const float                     cover_shrink = 4e-4f, other_shrink = 2e-4f;
    cuda_tool::DeviceBuffer<AABB>   d_grown[4];
    cuda_tool::DeviceBuffer<IndexT> ref_escaped, escaped;
    std::vector<IndexT>             flags = {7, 7, 7};
    ref_escaped.copy_from(flags.data(), 3);
    escaped.copy_from(flags.data(), 3);
    if(contained)
    {
        std::uniform_real_distribution<float> skin(-1e-4f, 1e-3f);
        for(int f = 0; f < 4; ++f)
        {
            auto boxes = download(ref[f].aabbs);
            for(auto& b : boxes)
            {
                float s = skin(rng);
                b.min().array() -= s;
                b.max().array() += s;
            }
            d_grown[f].copy_from(boxes.data(), boxes.size());
        }
        std::vector<IndexT> zero = {0, 0, 0};
        ref_escaped.copy_from(zero.data(), 3);
        for(int f = 0; f < 4; ++f)
            if(n[f] > 0)
                trajectory_prefix_test_contained_kernel<<<grid(n[f]), TestBlock>>>(
                    ref[f].aabbs.view(),
                    d_grown[f].view(),
                    f == 0 ? cuda_tool::CBufferView<IndexT>{} : d_is_query[f].view(),
                    cover_shrink,
                    other_shrink,
                    ref_escaped.view(),
                    n[f]);
    }

    // The prefix.
    TrajectoryPrefix p;
    p.positions           = d_Ps.data();
    p.displacements       = d_dxs.data();
    p.d_hats              = d_dh.data();
    p.thicknesses         = d_th.data();
    p.v2bs                = d_v2b.data();
    p.contact_ids         = d_cid.data();
    p.vertices            = nv;
    p.alpha               = alpha;
    p.f32_positions       = pos.data();
    p.f32_steps           = steps.data();
    p.f32_ends            = ends.data();
    p.escaped             = contained ? escaped.data() : nullptr;
    p.cover_shrink        = cover_shrink;
    p.other_shrink        = other_shrink;
    p.family[0].vertices  = d_codim.data();
    p.family[1].vertices  = d_Vs.data();
    p.family[2].edges     = d_Es.data();
    p.family[3].triangles = d_Fs.data();
    for(int f = 0; f < 4; ++f)
    {
        auto& F = p.family[f];
        F.n     = n[f];
        F.aabbs = out[f].aabbs.data();
        F.bids  = out[f].bids.data();
        F.cids  = out[f].cids.data();
        if(f > 0)
        {
            F.slots          = d_slots[f].data();
            F.gathered_aabbs = out[f].gathered_aabbs.data();
            F.gathered_bids  = out[f].gathered_bids.data();
            F.gathered_cids  = out[f].gathered_cids.data();
        }
        if(contained)
        {
            F.grown    = d_grown[f].data();
            F.is_query = f > 0 ? d_is_query[f].data() : nullptr;
        }
    }
    launch_trajectory_prefix(p, nullptr);
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());

    for(int f = 0; f < 4; ++f)
    {
        INFO("family " << f << ", n " << n[f] << ", alpha " << alpha);
        REQUIRE(same_bytes(out[f].aabbs, ref[f].aabbs));
        REQUIRE(same_bytes(out[f].bids, ref[f].bids));
        REQUIRE(same_bytes(out[f].cids, ref[f].cids));
        REQUIRE(same_bytes(out[f].gathered_aabbs, ref[f].gathered_aabbs));
        REQUIRE(same_bytes(out[f].gathered_bids, ref[f].gathered_bids));
        REQUIRE(same_bytes(out[f].gathered_cids, ref[f].gathered_cids));
    }
    REQUIRE(same_bytes(pos, ref_pos));
    REQUIRE(same_bytes(steps, ref_steps));
    REQUIRE(same_bytes(escaped, ref_escaped));
    if(contained && ncodim + npoints + nedges + ntris > 100)  // some boxes escape
    {
        auto e = download(escaped);
        REQUIRE(e[0] == 1);
    }
}
}  // namespace

TEST_CASE("trajectory prefix writes the separate kernels' boxes, gathers, FP32 inputs and flags",
          "[cuda][collision detection][trajectory prefix]")
{
    check_trajectory_prefix(0, 0, 0, 0, 0, 1.0, false, 1);
    check_trajectory_prefix(0, 0, 0, 0, 0, 1.0, true, 2);
    check_trajectory_prefix(5, 0, 3, 4, 2, 0.5, true, 3);
    check_trajectory_prefix(3000, 0, 2900, 8000, 5500, 1.0, true, 4);
    check_trajectory_prefix(3000, 40, 2900, 8000, 5500, 0.0, true, 5);
    check_trajectory_prefix(3000, 40, 2900, 8000, 5500, 0.37, false, 6);
    check_trajectory_prefix(40000, 300, 39000, 117000, 78000, 0.731, true, 7);
    check_trajectory_prefix(1024, 1, 1023, 1025, 769, 1e-7, true, 8);
}
