/*
 * NVIDIA contributions: BVH trajectory-query, scene-bounds, traversal and
 * refit regression coverage, and the side-stream build/refit-query parity and
 * warp-cooperative Other-query traversal tests.
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

#include <type_define.h>
#include <app/app.h>
#include <app/asset_dir.h>
#include <collision_detection/info_stackless_bvh.h>
#include <uipc/geometry.h>
#include <uipc/uipc.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <list>
#include <utility>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace cuda_tool;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;

namespace test_info_stackless_bvh
{
struct NodePred
{
    UIPC_GENERIC bool operator()(const InfoStacklessBVH::NodePredInfo&) const
    {
        return true;
    }
};

struct LeafPred
{
    UIPC_GENERIC bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
    {
        return ((info.i ^ info.j) & 1) == 0;
    }
};

struct CountingNodeCull
{
    int* calls;

    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo&) const
    {
        atomicAdd(calls, 1);
        return false;
    }
};

struct CountingLeafPair
{
    int* leaf_calls;

    __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo&) const
    {
        atomicAdd(leaf_calls, 1);
        return true;
    }
};

struct CullRateNodeCull
{
    int* calls;
    int* rejects;

    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        atomicAdd(calls, 1);
        bool keep = (info.query_id % 3) != 0;
        if(!keep)
            atomicAdd(rejects, 1);
        return keep;
    }
};

struct LeafPredTrue
{
    __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo&) const
    {
        return true;
    }
};

struct BidOnlyNodeCull
{
    static constexpr IndexT invalid = static_cast<IndexT>(-1);

    cuda_tool::BufferView<IndexT> bids;
    cuda_tool::BufferView<IndexT> is_self_contact;
    IndexT                        self_contact_count;
    int*                          calls;
    int*                          rejects;
    int*                          invalid_cid_hits;

    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        atomicAdd(calls, 1);
        if(info.node_cid == invalid)
            atomicAdd(invalid_cid_hits, 1);
        auto qbid = bids(info.query_id);
        bool self_contact_disabled = (qbid != invalid) && (qbid < self_contact_count)
                                     && !is_self_contact(qbid);
        bool keep = !(info.node_bid != invalid && qbid != invalid
                      && info.node_bid == qbid && self_contact_disabled);
        if(!keep)
            atomicAdd(rejects, 1);
        return keep;
    }
};

struct CidOnlyNodeCull
{
    static constexpr IndexT invalid = static_cast<IndexT>(-1);

    cuda_tool::BufferView<IndexT> bids;
    cuda_tool::BufferView<IndexT> cids;
    cuda_tool::Dense2D<IndexT>    cmts;
    cuda_tool::BufferView<IndexT> is_self_contact;
    IndexT                        self_contact_count;
    int*                          rejects;

    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        auto qbid = bids(info.query_id);
        auto qcid = cids(info.query_id);
        bool bid_cull = info.node_bid != invalid && qbid != invalid && info.node_bid == qbid
                        && (qbid < self_contact_count) && !is_self_contact(qbid);
        bool cid_cull = info.node_cid != invalid && qcid != invalid
                        && !cmts(qcid, info.node_cid);
        bool keep = !(bid_cull || cid_cull);
        if(!keep)
            atomicAdd(rejects, 1);
        return keep;
    }
};

struct FallbackNodeCull
{
    static constexpr IndexT invalid = static_cast<IndexT>(-1);

    cuda_tool::BufferView<IndexT> bids;
    cuda_tool::BufferView<IndexT> cids;
    cuda_tool::Dense2D<IndexT>    cmts;
    cuda_tool::BufferView<IndexT> is_self_contact;
    IndexT                        self_contact_count;
    int*                          rejects;
    int*                          invalid_bid_hits;
    int*                          invalid_cid_hits;

    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        if(info.node_bid == invalid)
            atomicAdd(invalid_bid_hits, 1);
        if(info.node_cid == invalid)
            atomicAdd(invalid_cid_hits, 1);
        auto qbid = bids(info.query_id);
        auto qcid = cids(info.query_id);
        bool bid_cull = info.node_bid != invalid && qbid != invalid && info.node_bid == qbid
                        && (qbid < self_contact_count) && !is_self_contact(qbid);
        bool cid_cull = info.node_cid != invalid && qcid != invalid
                        && !cmts(qcid, info.node_cid);
        bool keep = !(bid_cull || cid_cull);
        if(!keep)
            atomicAdd(rejects, 1);
        return keep;
    }
};

struct CountingLeafPredFalse
{
    int* leaf_calls;

    __device__ bool operator()(InfoStacklessBVH::LeafPredInfo) const
    {
        atomicAdd(leaf_calls, 1);
        return false;
    }
};

void check_cp_conservative(span<Vector2i> test, span<Vector2i> gt)
{
    auto compare = [](const Vector2i& lhs, const Vector2i& rhs)
    { return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]); };

    std::ranges::sort(test, compare);
    std::ranges::sort(gt, compare);

    std::list<Vector2i> diff;
    std::set_difference(
        gt.begin(), gt.end(), test.begin(), test.end(), std::back_inserter(diff), compare);

    CHECK(diff.empty());
}

void check_cp_exact(std::vector<Vector2i> test, std::vector<Vector2i> gt)
{
    auto compare = [](const Vector2i& lhs, const Vector2i& rhs)
    { return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]); };

    std::ranges::sort(test, compare);
    std::ranges::sort(gt, compare);
    REQUIRE(test.size() == gt.size());
    for(size_t i = 0; i < gt.size(); ++i)
    {
        CAPTURE(i);
        CHECK(test[i][0] == gt[i][0]);
        CHECK(test[i][1] == gt[i][1]);
    }
}

bool allow_contact(span<const IndexT> cmts, IndexT cid_count, IndexT lhs, IndexT rhs)
{
    return cmts[lhs * cid_count + rhs] != 0;
}

std::vector<Vector2i> brute_force_detect(span<const AABB>   aabbs,
                                         span<const IndexT> bids,
                                         span<const IndexT> cids,
                                         span<const IndexT> cmts,
                                         IndexT             cid_count)
{
    std::vector<Vector2i> pairs;
    LeafPred              lp;

    for(IndexT i = 0; i < static_cast<IndexT>(aabbs.size()); ++i)
    {
        for(IndexT j = i + 1; j < static_cast<IndexT>(aabbs.size()); ++j)
        {
            if(!aabbs[i].intersects(aabbs[j]))
                continue;
            if(bids[i] == bids[j])
                continue;
            if(!allow_contact(cmts, cid_count, cids[i], cids[j]))
                continue;
            if(!lp(InfoStacklessBVH::LeafPredInfo{i, j, bids[i], cids[i], bids[j], cids[j]}))
                continue;

            pairs.emplace_back(i, j);
        }
    }

    return pairs;
}

std::vector<Vector2i> brute_force_query(span<const AABB>   query_aabbs,
                                        span<const IndexT> query_bids,
                                        span<const IndexT> query_cids,
                                        span<const AABB>   tree_aabbs,
                                        span<const IndexT> tree_bids,
                                        span<const IndexT> tree_cids,
                                        span<const IndexT> cmts,
                                        IndexT             cid_count)
{
    std::vector<Vector2i> pairs;
    LeafPred              lp;

    for(IndexT i = 0; i < static_cast<IndexT>(query_aabbs.size()); ++i)
    {
        for(IndexT j = 0; j < static_cast<IndexT>(tree_aabbs.size()); ++j)
        {
            if(!query_aabbs[i].intersects(tree_aabbs[j]))
                continue;
            if(query_bids[i] == tree_bids[j])
                continue;
            if(!allow_contact(cmts, cid_count, query_cids[i], tree_cids[j]))
                continue;
            if(!lp(InfoStacklessBVH::LeafPredInfo{
                   i, j, query_bids[i], query_cids[i], tree_bids[j], tree_cids[j]}))
                continue;

            pairs.emplace_back(i, j);
        }
    }

    return pairs;
}

void run_info_stackless_bvh_test(const SimplicialComplex& mesh)
{
    auto pos_view = mesh.positions().view();
    auto tri_view = mesh.triangles().topo().view();

    std::vector<AABB>   aabbs(tri_view.size());
    std::vector<IndexT> bids(tri_view.size());
    std::vector<IndexT> cids(tri_view.size());

    for(auto&& [i, tri] : enumerate(tri_view))
    {
        auto p0 = pos_view[tri[0]];
        auto p1 = pos_view[tri[1]];
        auto p2 = pos_view[tri[2]];
        aabbs[i].extend(p0.cast<float>()).extend(p1.cast<float>()).extend(p2.cast<float>());

        bids[i] = static_cast<IndexT>(i % 7);
        cids[i] = static_cast<IndexT>(i % 4);
    }

    constexpr IndexT    cid_count = 4;
    std::vector<IndexT> cmts(cid_count * cid_count, 1);
    for(IndexT i = 0; i < cid_count; ++i)
    {
        for(IndexT j = 0; j < cid_count; ++j)
        {
            cmts[i * cid_count + j] = (((i + j) % 3) != 0) ? 1 : 0;
        }
    }

    DeviceBuffer<AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());
    DeviceBuffer<IndexT> d_bids(bids.size());
    d_bids.view().copy_from(bids.data());
    DeviceBuffer<IndexT> d_cids(cids.size());
    d_cids.view().copy_from(cids.data());
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{static_cast<size_t>(cid_count),
                                           static_cast<size_t>(cid_count)});
    d_cmts.view().copy_from(cmts.data());

    InfoStacklessBVH bvh;
    bvh.build(d_aabbs, d_bids, d_cids);

    InfoStacklessBVH::QueryBuffer qbuffer;
    qbuffer.reserve(1024);

    {
        Timer timer("info_stackless_bvh detect");
        bvh.detect(d_cmts.view(), NodePred{}, LeafPred{}, qbuffer);
    }

    std::vector<Vector2i> detect_pairs(qbuffer.size());
    qbuffer.view().copy_to(detect_pairs.data());
    auto detect_gt = brute_force_detect(aabbs, bids, cids, cmts, cid_count);
    check_cp_conservative(detect_pairs, detect_gt);

    // Exercise the launch/finalize split and its overflow retry with a queue
    // deliberately smaller than the expected result.
    InfoStacklessBVH::QueryBuffer batched_qbuffer;
    batched_qbuffer.m_pairs.release();
    batched_qbuffer.reserve(1);
    bvh.launch_detect(d_cmts.view(), NodePred{}, LeafPred{}, batched_qbuffer);
    int  batched_count = batched_qbuffer.m_cpNum;
    bool retry = bvh.prepare_query_result(batched_qbuffer, batched_count);
    if(retry)
        bvh.launch_detect(d_cmts.view(), NodePred{}, LeafPred{}, batched_qbuffer);

    std::vector<Vector2i> batched_pairs(batched_qbuffer.size());
    batched_qbuffer.view().copy_to(batched_pairs.data());
    check_cp_conservative(batched_pairs, detect_gt);
    CHECK(retry == (batched_count > 1));

    std::vector<AABB>   query_aabbs = aabbs;
    std::vector<IndexT> query_bids  = bids;
    std::vector<IndexT> query_cids  = cids;

    for(IndexT i = 0; i < static_cast<IndexT>(query_bids.size()); ++i)
    {
        query_bids[i] = (query_bids[i] + 3) % 11;
    }

    DeviceBuffer<AABB> d_query_aabbs(query_aabbs.size());
    d_query_aabbs.view().copy_from(query_aabbs.data());
    DeviceBuffer<IndexT> d_query_bids(query_bids.size());
    d_query_bids.view().copy_from(query_bids.data());
    DeviceBuffer<IndexT> d_query_cids(query_cids.size());
    d_query_cids.view().copy_from(query_cids.data());

    {
        Timer timer("info_stackless_bvh query");
        bvh.query(d_query_aabbs.view(),
                  d_query_bids.view(),
                  d_query_cids.view(),
                  d_cmts.view(),
                  NodePred{},
                  LeafPred{},
                  qbuffer);
    }

    std::vector<Vector2i> query_pairs(qbuffer.size());
    qbuffer.view().copy_to(query_pairs.data());
    auto query_gt = brute_force_query(
        query_aabbs, query_bids, query_cids, aabbs, bids, cids, cmts, cid_count);
    check_cp_conservative(query_pairs, query_gt);
}

void run_internal_cull_proof_case()
{
    constexpr IndexT    n = 64;
    std::vector<AABB>   aabbs(n);
    std::vector<IndexT> bids(n, 3);
    std::vector<IndexT> cids(n, 0);
    for(IndexT i = 0; i < n; ++i)
    {
        double  x  = static_cast<double>(i) * 1.0e-3;
        Vector3 p0 = Vector3{x, x, x};
        Vector3 p1 = Vector3{x + 1.0, x + 1.0, x + 1.0};
        AABB    box;
        box.extend(p0.cast<float>()).extend(p1.cast<float>());
        aabbs[i] = box;
    }

    DeviceBuffer<AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());
    DeviceBuffer<IndexT> d_bids(bids.size());
    d_bids.view().copy_from(bids.data());
    DeviceBuffer<IndexT> d_cids(cids.size());
    d_cids.view().copy_from(cids.data());
    InfoStacklessBVH::Impl impl;
    impl.build(d_aabbs.view(), d_bids.view(), d_cids.view());

    DeviceVar<int>         cp_num;
    DeviceBuffer<int>      node_cull_calls(1);
    DeviceBuffer<int>      leaf_pair_calls(1);
    DeviceBuffer<Vector2i> pairs(16);
    BufferLaunch().fill(cp_num.view(), 0);
    BufferLaunch().fill(node_cull_calls.view(), 0);
    BufferLaunch().fill(leaf_pair_calls.view(), 0);

    impl.stacklessSelf(CountingNodeCull{node_cull_calls.data()},
                       CountingLeafPair{leaf_pair_calls.data()},
                       cp_num.view(),
                       pairs.view());

    int h_node_cull_calls = 0;
    int h_leaf_pair_calls = 0;
    node_cull_calls.view(0, 1).copy_to(&h_node_cull_calls);
    leaf_pair_calls.view(0, 1).copy_to(&h_leaf_pair_calls);
    int h_pairs = cp_num;

    fmt::println("internal-cull proof stats: node_cull_calls={}, leaf_pair_calls={}, pairs={}",
                 h_node_cull_calls,
                 h_leaf_pair_calls,
                 h_pairs);

    // The final Morton-rank query skips the root because no greater-ranked
    // leaf can exist below it. All other queries reach and reject the root.
    CHECK(h_node_cull_calls == n - 1);
    CHECK(h_leaf_pair_calls == 0);
    CHECK(h_pairs == 0);
}

void run_self_rank_reordered_node_case()
{
    // Raw ids 0,1,2,3 have Morton order 1,3,2,0. Distinct centers avoid any
    // dependence on radix-sort tie order; the common extent overlaps all pairs.
    constexpr std::array<float, 4> x_by_raw_id = {3.0f, 0.0f, 2.0f, 1.0f};
    std::vector<AABB>              aabbs;
    for(float x : x_by_raw_id)
    {
        AABB box;
        box.extend(Vector3{x, 0.0, 0.0}.cast<float>());
        box.extend(Vector3{x + 4.0, 4.0, 4.0}.cast<float>());
        aabbs.push_back(box);
    }

    std::vector<IndexT>  bids(aabbs.size(), 0);
    std::vector<IndexT>  cids(aabbs.size(), 0);
    DeviceBuffer<AABB>   d_aabbs(aabbs.size());
    DeviceBuffer<IndexT> d_bids(bids.size());
    DeviceBuffer<IndexT> d_cids(cids.size());
    d_aabbs.view().copy_from(aabbs.data());
    d_bids.view().copy_from(bids.data());
    d_cids.view().copy_from(cids.data());

    DeviceBuffer2D<IndexT> d_cmts(Extent2D{1, 1});
    IndexT                 allow = 1;
    d_cmts.view().copy_from(&allow);

    InfoStacklessBVH bvh;
    bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
    InfoStacklessBVH::QueryBuffer qbuffer;
    qbuffer.reserve(8);
    bvh.detect(d_cmts.view(), NodePred{}, LeafPredTrue{}, qbuffer);

    std::vector<Vector2i> actual(qbuffer.size());
    qbuffer.view().copy_to(actual.data());
    std::vector<Vector2i> expected;
    for(IndexT i = 0; i < static_cast<IndexT>(aabbs.size()); ++i)
        for(IndexT j = i + 1; j < static_cast<IndexT>(aabbs.size()); ++j)
            expected.emplace_back(i, j);
    check_cp_exact(std::move(actual), std::move(expected));
}

std::vector<AABB> make_all_overlapping_aabbs(size_t count)
{
    std::vector<AABB> aabbs(count);
    for(size_t i = 0; i < count; ++i)
    {
        const float shift = static_cast<float>(i) * 1.0e-4f;
        aabbs[i].extend(Vector3{shift, shift, shift}.cast<float>());
        aabbs[i].extend(Vector3{1.0 + shift, 1.0 + shift, 1.0 + shift}.cast<float>());
    }
    return aabbs;
}

std::vector<Vector2i> run_self_all_pairs(const std::vector<AABB>& aabbs, bool& retried)
{
    std::vector<IndexT>  bids(aabbs.size(), 0);
    std::vector<IndexT>  cids(aabbs.size(), 0);
    DeviceBuffer<AABB>   d_aabbs(aabbs.size());
    DeviceBuffer<IndexT> d_bids(bids.size());
    DeviceBuffer<IndexT> d_cids(cids.size());
    if(!aabbs.empty())
    {
        d_aabbs.view().copy_from(aabbs.data());
        d_bids.view().copy_from(bids.data());
        d_cids.view().copy_from(cids.data());
    }

    DeviceBuffer2D<IndexT> d_cmts(Extent2D{1, 1});
    IndexT                 allow = 1;
    d_cmts.view().copy_from(&allow);

    InfoStacklessBVH bvh;
    bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
    InfoStacklessBVH::QueryBuffer qbuffer;
    qbuffer.m_pairs.release();
    qbuffer.reserve(1);
    bvh.launch_detect(d_cmts.view(), NodePred{}, LeafPredTrue{}, qbuffer);
    int count = qbuffer.m_cpNum;
    retried   = bvh.prepare_query_result(qbuffer, count);
    if(retried)
        bvh.launch_detect(d_cmts.view(), NodePred{}, LeafPredTrue{}, qbuffer);

    std::vector<Vector2i> pairs(qbuffer.size());
    if(!pairs.empty())
        qbuffer.view().copy_to(pairs.data());
    return pairs;
}

std::vector<Vector2i> run_other_all_pairs(const std::vector<AABB>& query_aabbs,
                                          const std::vector<AABB>& tree_aabbs,
                                          bool&                    retried)
{
    std::vector<IndexT> query_bids(query_aabbs.size(), 0);
    std::vector<IndexT> query_cids(query_aabbs.size(), 0);
    std::vector<IndexT> tree_bids(tree_aabbs.size(), 0);
    std::vector<IndexT> tree_cids(tree_aabbs.size(), 0);

    DeviceBuffer<AABB>   d_query_aabbs(query_aabbs.size());
    DeviceBuffer<IndexT> d_query_bids(query_bids.size());
    DeviceBuffer<IndexT> d_query_cids(query_cids.size());
    if(!query_aabbs.empty())
    {
        d_query_aabbs.view().copy_from(query_aabbs.data());
        d_query_bids.view().copy_from(query_bids.data());
        d_query_cids.view().copy_from(query_cids.data());
    }

    DeviceBuffer<AABB>   d_tree_aabbs(tree_aabbs.size());
    DeviceBuffer<IndexT> d_tree_bids(tree_bids.size());
    DeviceBuffer<IndexT> d_tree_cids(tree_cids.size());
    d_tree_aabbs.view().copy_from(tree_aabbs.data());
    d_tree_bids.view().copy_from(tree_bids.data());
    d_tree_cids.view().copy_from(tree_cids.data());

    DeviceBuffer2D<IndexT> d_cmts(Extent2D{1, 1});
    IndexT                 allow = 1;
    d_cmts.view().copy_from(&allow);

    InfoStacklessBVH bvh;
    bvh.build(d_tree_aabbs.view(), d_tree_bids.view(), d_tree_cids.view());
    InfoStacklessBVH::QueryBuffer qbuffer;
    qbuffer.m_pairs.release();
    qbuffer.reserve(1);
    bvh.launch_query(d_query_aabbs.view(),
                     d_query_bids.view(),
                     d_query_cids.view(),
                     d_cmts.view(),
                     NodePred{},
                     LeafPredTrue{},
                     qbuffer,
                     true);
    int count = qbuffer.m_cpNum;
    retried   = bvh.prepare_query_result(qbuffer, count);
    if(retried)
        bvh.launch_query(d_query_aabbs.view(),
                         d_query_bids.view(),
                         d_query_cids.view(),
                         d_cmts.view(),
                         NodePred{},
                         LeafPredTrue{},
                         qbuffer,
                         false);

    std::vector<Vector2i> pairs(qbuffer.size());
    if(!pairs.empty())
        qbuffer.view().copy_to(pairs.data());
    return pairs;
}

void run_cta_queue_boundary_cases()
{
    constexpr std::array<size_t, 14> counts = {
        0, 1, 31, 32, 33, 63, 64, 65, 127, 128, 129, 255, 256, 257};

    for(size_t count : counts)
    {
        DYNAMIC_SECTION("Self N=" << count)
        {
            auto                  aabbs = make_all_overlapping_aabbs(count);
            std::vector<Vector2i> expected;
            for(IndexT i = 0; i < static_cast<IndexT>(count); ++i)
                for(IndexT j = i + 1; j < static_cast<IndexT>(count); ++j)
                    expected.emplace_back(i, j);
            bool retried = false;
            auto actual  = run_self_all_pairs(aabbs, retried);
            CHECK(retried == (expected.size() > 1));
            check_cp_exact(std::move(actual), std::move(expected));
        }
    }

    const auto tree_aabbs = make_all_overlapping_aabbs(65);
    for(size_t count : counts)
    {
        DYNAMIC_SECTION("Other N=" << count)
        {
            auto query_aabbs = make_all_overlapping_aabbs(count);
            std::vector<Vector2i> expected;
            for(IndexT i = 0; i < static_cast<IndexT>(count); ++i)
                for(IndexT j = 0; j < static_cast<IndexT>(tree_aabbs.size()); ++j)
                    expected.emplace_back(i, j);
            bool retried = false;
            auto actual = run_other_all_pairs(query_aabbs, tree_aabbs, retried);
            CHECK(retried == (expected.size() > 1));
            check_cp_exact(std::move(actual), std::move(expected));
        }
    }
}

struct RefitNodePred
{
    cuda_tool::CDense2D<IndexT> cmts;

    UIPC_GENERIC bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        constexpr IndexT invalid = static_cast<IndexT>(-1);
        bool bid_cull = info.query_bid != invalid && info.node_bid != invalid
                        && info.query_bid == info.node_bid;
        bool cid_cull = info.query_cid != invalid && info.node_cid != invalid
                        && !cmts(info.query_cid, info.node_cid);
        return !(bid_cull || cid_cull);
    }
};

struct RefitLeafPred
{
    cuda_tool::CDense2D<IndexT> cmts;

    UIPC_GENERIC bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
    {
        return info.bid_i != info.bid_j && cmts(info.cid_i, info.cid_j);
    }
};

struct RefitInputs
{
    std::vector<AABB>   aabbs;
    std::vector<IndexT> bids;
    std::vector<IndexT> cids;
};

AABB refit_box_at(double x, double radius)
{
    AABB box;
    box.extend(Vector3{x - radius, -radius, -radius}.cast<float>());
    box.extend(Vector3{x + radius, radius, radius}.cast<float>());
    return box;
}

RefitInputs refit_initial_inputs(IndexT count)
{
    RefitInputs result;
    result.aabbs.reserve(count);
    result.bids.reserve(count);
    result.cids.reserve(count);
    for(IndexT i = 0; i < count; ++i)
    {
        result.aabbs.push_back(refit_box_at(3.0 * i, 0.25));
        result.bids.push_back(i % 7);
        result.cids.push_back(i % 4);
    }
    return result;
}

RefitInputs refit_swept_inputs(IndexT count, double phase)
{
    RefitInputs result;
    result.aabbs.reserve(count);
    result.bids.reserve(count);
    result.cids.reserve(count);
    for(IndexT i = 0; i < count; ++i)
    {
        // Reverse spatial order and overlap many leaves so a stale or
        // incompletely published parent becomes visible in the exact pairs.
        double x = 0.015 * (count - 1 - i) + phase * ((i % 3) - 1);
        result.aabbs.push_back(refit_box_at(x, 0.65));
        result.bids.push_back((i + 3) % 9);
        result.cids.push_back((i + 1) % 4);
    }
    return result;
}

enum class RefitMetadataPhase
{
    Homogeneous,
    DifferentHomogeneous,
    Mixed
};

RefitInputs refit_grouped_inputs(IndexT count, RefitMetadataPhase phase, bool reverse_groups)
{
    constexpr IndexT group_size = 3;
    REQUIRE(count % group_size == 0);

    RefitInputs result;
    result.aabbs.reserve(count);
    result.bids.reserve(count);
    result.cids.reserve(count);
    IndexT group_count = count / group_size;
    for(IndexT i = 0; i < count; ++i)
    {
        IndexT group         = i / group_size;
        IndexT local         = i % group_size;
        IndexT spatial_group = reverse_groups ? group_count - 1 - group : group;
        result.aabbs.push_back(refit_box_at(10.0 * spatial_group + 0.15 * local, 0.2));

        switch(phase)
        {
            case RefitMetadataPhase::Homogeneous:
                result.bids.push_back(17);
                result.cids.push_back(1);
                break;
            case RefitMetadataPhase::DifferentHomogeneous:
                result.bids.push_back(29);
                result.cids.push_back(2);
                break;
            case RefitMetadataPhase::Mixed:
                result.bids.push_back(i);
                result.cids.push_back(local);
                break;
        }
    }
    return result;
}

RefitInputs refit_grouped_queries(IndexT group_count, IndexT bid, IndexT cid)
{
    RefitInputs result;
    result.aabbs.reserve(group_count);
    result.bids.reserve(group_count);
    result.cids.reserve(group_count);
    for(IndexT group = 0; group < group_count; ++group)
    {
        result.aabbs.push_back(refit_box_at(10.0 * group + 0.15, 0.4));
        result.bids.push_back(bid);
        result.cids.push_back(cid);
    }
    return result;
}

void upload_refit(const RefitInputs&    input,
                  DeviceBuffer<AABB>&   aabbs,
                  DeviceBuffer<IndexT>& bids,
                  DeviceBuffer<IndexT>& cids)
{
    aabbs.resize(input.aabbs.size());
    bids.resize(input.bids.size());
    cids.resize(input.cids.size());
    if(!input.aabbs.empty())
    {
        aabbs.view().copy_from(input.aabbs.data());
        bids.view().copy_from(input.bids.data());
        cids.view().copy_from(input.cids.data());
    }
}

std::vector<Vector2i> refit_detect_pairs(InfoStacklessBVH&              bvh,
                                         CBuffer2DView<IndexT>          cmts,
                                         InfoStacklessBVH::QueryBuffer& pairs)
{
    bvh.detect(cmts, RefitNodePred{cmts.viewer()}, RefitLeafPred{cmts.viewer()}, pairs);

    std::vector<Vector2i> result(pairs.size());
    if(!result.empty())
        pairs.view().copy_to(result.data());
    return result;
}

std::vector<Vector2i> refit_detect_pairs(InfoStacklessBVH& bvh, CBuffer2DView<IndexT> cmts)
{
    InfoStacklessBVH::QueryBuffer pairs;
    pairs.m_pairs.release();
    pairs.reserve(1);
    return refit_detect_pairs(bvh, cmts, pairs);
}

std::vector<Vector2i> refit_query_pairs(InfoStacklessBVH&     bvh,
                                        CBufferView<AABB>     query_aabbs,
                                        CBufferView<IndexT>   query_bids,
                                        CBufferView<IndexT>   query_cids,
                                        CBuffer2DView<IndexT> cmts)
{
    InfoStacklessBVH::QueryBuffer pairs;
    pairs.m_pairs.release();
    pairs.reserve(1);
    bvh.query(query_aabbs,
              query_bids,
              query_cids,
              cmts,
              RefitNodePred{cmts.viewer()},
              RefitLeafPred{cmts.viewer()},
              pairs);

    std::vector<Vector2i> result(pairs.size());
    if(!result.empty())
        pairs.view().copy_to(result.data());
    return result;
}

std::vector<Vector2i> refit_query_pairs(InfoStacklessBVH&     bvh,
                                        CBufferView<AABB>     query_aabbs,
                                        CBufferView<IndexT>   query_bids,
                                        CBufferView<IndexT>   query_cids,
                                        CBuffer2DView<IndexT> cmts,
                                        InfoStacklessBVH::QueryBuffer& pairs,
                                        bool reuse_query_order)
{
    bvh.query(query_aabbs,
              query_bids,
              query_cids,
              cmts,
              RefitNodePred{cmts.viewer()},
              RefitLeafPred{cmts.viewer()},
              pairs,
              reuse_query_order);

    std::vector<Vector2i> result(pairs.size());
    if(!result.empty())
        pairs.view().copy_to(result.data());
    return result;
}

std::vector<int> copy_query_order(const InfoStacklessBVH::QueryBuffer& pairs)
{
    std::vector<int> result(pairs.m_querySortedId.size());
    if(!result.empty())
        pairs.m_querySortedId.view().copy_to(result.data());
    return result;
}

size_t check_refit_pairs(std::vector<Vector2i> refitted, std::vector<Vector2i> rebuilt)
{
    size_t count = refitted.size();
    check_cp_exact(std::move(refitted), std::move(rebuilt));
    return count;
}

void compare_full_build_and_refit(const RefitInputs&    initial,
                                  const RefitInputs&    swept,
                                  CBuffer2DView<IndexT> cmts)
{
    DeviceBuffer<AABB>   refit_aabbs;
    DeviceBuffer<IndexT> refit_bids;
    DeviceBuffer<IndexT> refit_cids;
    upload_refit(initial, refit_aabbs, refit_bids, refit_cids);

    InfoStacklessBVH refitted;
    refitted.build(refit_aabbs, refit_bids, refit_cids);
    upload_refit(swept, refit_aabbs, refit_bids, refit_cids);
    REQUIRE(refitted.refit(refit_aabbs, refit_bids, refit_cids));

    DeviceBuffer<AABB>   rebuilt_aabbs;
    DeviceBuffer<IndexT> rebuilt_bids;
    DeviceBuffer<IndexT> rebuilt_cids;
    upload_refit(swept, rebuilt_aabbs, rebuilt_bids, rebuilt_cids);
    InfoStacklessBVH rebuilt;
    rebuilt.build(rebuilt_aabbs, rebuilt_bids, rebuilt_cids);

    check_refit_pairs(refit_detect_pairs(refitted, cmts), refit_detect_pairs(rebuilt, cmts));

    auto                 query = refit_swept_inputs(11, 0.025);
    DeviceBuffer<AABB>   query_aabbs;
    DeviceBuffer<IndexT> query_bids;
    DeviceBuffer<IndexT> query_cids;
    upload_refit(query, query_aabbs, query_bids, query_cids);
    check_refit_pairs(
        refit_query_pairs(refitted, query_aabbs, query_bids, query_cids, cmts),
        refit_query_pairs(rebuilt, query_aabbs, query_bids, query_cids, cmts));
}

void run_refit_cases()
{
    constexpr IndexT    cid_count = 4;
    std::vector<IndexT> cmts(cid_count * cid_count);
    for(IndexT i = 0; i < cid_count; ++i)
        for(IndexT j = 0; j < cid_count; ++j)
            cmts[i * cid_count + j] = ((i + j) % 3) != 0;
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{cid_count, cid_count});
    d_cmts.view().copy_from(cmts.data());

    SECTION("empty")
    {
        DeviceBuffer<AABB>   aabbs;
        DeviceBuffer<IndexT> bids;
        DeviceBuffer<IndexT> cids;
        InfoStacklessBVH     bvh;
        bvh.build(aabbs, bids, cids);
        CHECK(bvh.refit(aabbs, bids, cids));
        CHECK(refit_detect_pairs(bvh, d_cmts.view()).empty());
    }

    SECTION("single primitive")
    {
        compare_full_build_and_refit(
            refit_initial_inputs(1), refit_swept_inputs(1, 0.0), d_cmts.view());
    }

    SECTION("swept topology and metadata")
    {
        compare_full_build_and_refit(refit_initial_inputs(33),
                                     refit_swept_inputs(33, 0.0),
                                     d_cmts.view());
    }

    SECTION("many Newton refits match fresh builds exactly")
    {
        constexpr IndexT primitive_count = 96;
        constexpr IndexT group_size      = 3;
        constexpr IndexT newton_steps    = 16;

        DeviceBuffer<AABB>   refit_aabbs;
        DeviceBuffer<IndexT> refit_bids;
        DeviceBuffer<IndexT> refit_cids;
        upload_refit(refit_grouped_inputs(primitive_count, RefitMetadataPhase::Homogeneous, false),
                     refit_aabbs,
                     refit_bids,
                     refit_cids);
        InfoStacklessBVH refitted;
        refitted.build(refit_aabbs, refit_bids, refit_cids);

        DeviceBuffer<AABB>   rebuilt_aabbs;
        DeviceBuffer<IndexT> rebuilt_bids;
        DeviceBuffer<IndexT> rebuilt_cids;
        InfoStacklessBVH     rebuilt;

        DeviceBuffer<AABB>   query_aabbs;
        DeviceBuffer<IndexT> query_bids;
        DeviceBuffer<IndexT> query_cids;

        InfoStacklessBVH::QueryBuffer refitted_self_pairs;
        InfoStacklessBVH::QueryBuffer rebuilt_self_pairs;
        InfoStacklessBVH::QueryBuffer refitted_query_pairs;
        InfoStacklessBVH::QueryBuffer rebuilt_query_pairs;
        for(auto* pairs :
            {&refitted_self_pairs, &rebuilt_self_pairs, &refitted_query_pairs, &rebuilt_query_pairs})
        {
            pairs->m_pairs.release();
            pairs->reserve(1);
        }

        size_t exercised_pair_count = 0;
        for(IndexT newton_step = 1; newton_step <= newton_steps; ++newton_step)
        {
            RefitMetadataPhase phase = RefitMetadataPhase::Mixed;
            switch(newton_step % 3)
            {
                case 0:
                    phase = RefitMetadataPhase::Homogeneous;
                    break;
                case 1:
                    phase = RefitMetadataPhase::DifferentHomogeneous;
                    break;
                default:
                    break;
            }

            // Alternate the spatial group order while also cycling leaf BID/CID
            // layouts. This crosses two K=8 windows without refreshing the
            // cached tree, making stale bounds or metadata publication visible.
            auto state =
                refit_grouped_inputs(primitive_count, phase, (newton_step & 1) != 0);
            upload_refit(state, refit_aabbs, refit_bids, refit_cids);
            CAPTURE(newton_step, phase);
            REQUIRE(refitted.refit(refit_aabbs, refit_bids, refit_cids));

            upload_refit(state, rebuilt_aabbs, rebuilt_bids, rebuilt_cids);
            rebuilt.build(rebuilt_aabbs, rebuilt_bids, rebuilt_cids);

            exercised_pair_count += check_refit_pairs(
                refit_detect_pairs(refitted, d_cmts.view(), refitted_self_pairs),
                refit_detect_pairs(rebuilt, d_cmts.view(), rebuilt_self_pairs));

            auto query = refit_grouped_queries(primitive_count / group_size,
                                               newton_step % 2 ? 17 : 29,
                                               newton_step % cid_count);
            upload_refit(query, query_aabbs, query_bids, query_cids);
            exercised_pair_count += check_refit_pairs(
                refit_query_pairs(
                    refitted, query_aabbs, query_bids, query_cids, d_cmts.view(), refitted_query_pairs, newton_step > 1),
                refit_query_pairs(
                    rebuilt, query_aabbs, query_bids, query_cids, d_cmts.view(), rebuilt_query_pairs, false));
        }

        CHECK(exercised_pair_count > 0);
    }

    SECTION("count changes fall back to exact rebuilds")
    {
        DeviceBuffer<AABB>   refit_aabbs;
        DeviceBuffer<IndexT> refit_bids;
        DeviceBuffer<IndexT> refit_cids;
        upload_refit(refit_initial_inputs(32), refit_aabbs, refit_bids, refit_cids);
        InfoStacklessBVH refitted;
        refitted.build(refit_aabbs, refit_bids, refit_cids);

        DeviceBuffer<AABB>   rebuilt_aabbs;
        DeviceBuffer<IndexT> rebuilt_bids;
        DeviceBuffer<IndexT> rebuilt_cids;
        InfoStacklessBVH     rebuilt;

        constexpr std::array<IndexT, 3> changed_counts = {33, 31, 32};
        for(auto&& [transition, count] : enumerate(changed_counts))
        {
            auto changed = refit_swept_inputs(count, 0.02 * transition);
            upload_refit(changed, refit_aabbs, refit_bids, refit_cids);
            CAPTURE(transition, count);
            REQUIRE_FALSE(refitted.refit(refit_aabbs, refit_bids, refit_cids));

            // Mirror the production `if(!refit) build` fallback and compare it
            // with an independently rebuilt tree at the identical state.
            refitted.build(refit_aabbs, refit_bids, refit_cids);
            upload_refit(changed, rebuilt_aabbs, rebuilt_bids, rebuilt_cids);
            rebuilt.build(rebuilt_aabbs, rebuilt_bids, rebuilt_cids);

            CHECK(check_refit_pairs(refit_detect_pairs(refitted, d_cmts.view()),
                                    refit_detect_pairs(rebuilt, d_cmts.view()))
                  > 0);
        }
    }

    SECTION("cross CTA metadata transitions and overflow")
    {
        constexpr IndexT primitive_count = 1536;
        constexpr IndexT group_size      = 3;
        static_assert(primitive_count > 1024);

        DeviceBuffer<AABB>   refit_aabbs;
        DeviceBuffer<IndexT> refit_bids;
        DeviceBuffer<IndexT> refit_cids;
        upload_refit(refit_grouped_inputs(primitive_count, RefitMetadataPhase::Homogeneous, false),
                     refit_aabbs,
                     refit_bids,
                     refit_cids);
        InfoStacklessBVH refitted;
        refitted.build(refit_aabbs, refit_bids, refit_cids);

        auto compare_refit_with_rebuild = [&](const RefitInputs& state, const RefitInputs& query)
        {
            upload_refit(state, refit_aabbs, refit_bids, refit_cids);
            REQUIRE(refitted.refit(refit_aabbs, refit_bids, refit_cids));

            DeviceBuffer<AABB>   rebuilt_aabbs;
            DeviceBuffer<IndexT> rebuilt_bids;
            DeviceBuffer<IndexT> rebuilt_cids;
            upload_refit(state, rebuilt_aabbs, rebuilt_bids, rebuilt_cids);
            InfoStacklessBVH rebuilt;
            rebuilt.build(rebuilt_aabbs, rebuilt_bids, rebuilt_cids);

            size_t self_count =
                check_refit_pairs(refit_detect_pairs(refitted, d_cmts.view()),
                                  refit_detect_pairs(rebuilt, d_cmts.view()));

            DeviceBuffer<AABB>   query_aabbs;
            DeviceBuffer<IndexT> query_bids;
            DeviceBuffer<IndexT> query_cids;
            upload_refit(query, query_aabbs, query_bids, query_cids);
            size_t other_count = check_refit_pairs(
                refit_query_pairs(
                    refitted, query_aabbs, query_bids, query_cids, d_cmts.view()),
                refit_query_pairs(
                    rebuilt, query_aabbs, query_bids, query_cids, d_cmts.view()));
            return std::pair{self_count, other_count};
        };

        auto different = refit_grouped_inputs(
            primitive_count, RefitMetadataPhase::DifferentHomogeneous, true);
        auto query_old_homogeneous =
            refit_grouped_queries(primitive_count / group_size, 17, 2);
        auto [different_self, different_other] =
            compare_refit_with_rebuild(different, query_old_homogeneous);
        CHECK(different_self == 0);
        CHECK(different_other > 1);

        auto mixed = refit_grouped_inputs(primitive_count, RefitMetadataPhase::Mixed, false);
        auto query_old_different =
            refit_grouped_queries(primitive_count / group_size, 29, 1);
        auto [mixed_self, mixed_other] =
            compare_refit_with_rebuild(mixed, query_old_different);
        CHECK(mixed_self > 1);
        CHECK(mixed_other > 1);
    }
}

void run_query_order_reuse_cases()
{
    constexpr IndexT    cid_count = 4;
    std::vector<IndexT> cmts(cid_count * cid_count);
    for(IndexT i = 0; i < cid_count; ++i)
        for(IndexT j = 0; j < cid_count; ++j)
            cmts[i * cid_count + j] = ((i + j) % 3) != 0;
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{cid_count, cid_count});
    d_cmts.view().copy_from(cmts.data());

    auto                 target = refit_swept_inputs(96, 0.0);
    DeviceBuffer<AABB>   target_aabbs;
    DeviceBuffer<IndexT> target_bids;
    DeviceBuffer<IndexT> target_cids;
    upload_refit(target, target_aabbs, target_bids, target_cids);
    InfoStacklessBVH bvh;
    bvh.build(target_aabbs, target_bids, target_cids);

    DeviceBuffer<AABB>            query_aabbs;
    DeviceBuffer<IndexT>          query_bids;
    DeviceBuffer<IndexT>          query_cids;
    InfoStacklessBVH::QueryBuffer reused;
    reused.m_pairs.release();
    reused.reserve(1);

    // Prime a Morton permutation, then move the same query IDs into reverse
    // spatial order. Reuse must keep the permutation while predicates still
    // see current boxes and produce the same exact pairs as a fresh sort.
    upload_refit(refit_initial_inputs(33), query_aabbs, query_bids, query_cids);
    refit_query_pairs(bvh, query_aabbs, query_bids, query_cids, d_cmts.view(), reused, false);
    auto initial_order = copy_query_order(reused);

    upload_refit(refit_swept_inputs(33, 0.04), query_aabbs, query_bids, query_cids);
    auto reused_pairs = refit_query_pairs(
        bvh, query_aabbs, query_bids, query_cids, d_cmts.view(), reused, true);
    CHECK(copy_query_order(reused) == initial_order);

    InfoStacklessBVH::QueryBuffer fresh;
    fresh.m_pairs.release();
    fresh.reserve(1);
    auto fresh_pairs = refit_query_pairs(
        bvh, query_aabbs, query_bids, query_cids, d_cmts.view(), fresh, false);
    auto fresh_order = copy_query_order(fresh);
    CHECK(initial_order != fresh_order);
    CHECK(check_refit_pairs(std::move(reused_pairs), std::move(fresh_pairs)) > 1);

    // A count change cannot reuse the old permutation and must rebuild it.
    upload_refit(refit_swept_inputs(34, 0.02), query_aabbs, query_bids, query_cids);
    auto changed_reused = refit_query_pairs(
        bvh, query_aabbs, query_bids, query_cids, d_cmts.view(), reused, true);
    InfoStacklessBVH::QueryBuffer changed_fresh;
    changed_fresh.m_pairs.release();
    changed_fresh.reserve(1);
    auto changed_fresh_pairs = refit_query_pairs(
        bvh, query_aabbs, query_bids, query_cids, d_cmts.view(), changed_fresh, false);
    CHECK(copy_query_order(reused) == copy_query_order(changed_fresh));
    CHECK(check_refit_pairs(std::move(changed_reused), std::move(changed_fresh_pairs)) > 1);
}

// Lattice boxes with jitter: each box overlaps its 26 lattice neighbors.
RefitInputs lattice_inputs(IndexT count, IndexT seed)
{
    RefitInputs result;
    result.aabbs.reserve(count);
    result.bids.reserve(count);
    result.cids.reserve(count);
    for(IndexT i = 0; i < count; ++i)
    {
        auto hash = static_cast<uint32_t>((i + 1) * 2654435761u + seed * 40503u);
        double jitter = 0.1 * ((hash % 1000) / 1000.0 - 0.5);
        Vector3 c{(i % 16) + jitter, ((i / 16) % 16) - jitter, i / 256 + 0.5 * jitter};
        AABB box;
        box.extend((c - Vector3::Constant(0.55)).cast<float>());
        box.extend((c + Vector3::Constant(0.55)).cast<float>());
        result.aabbs.push_back(box);
        result.bids.push_back((i + seed) % 7);
        result.cids.push_back((i + seed) % 4);
    }
    return result;
}

// Two independent refit-or-build -> query chains, each forked onto its own
// SideStream (as the trajectory filter runs its PT chains), must produce the
// same pair multisets as the same chains run on the default stream.
void run_side_stream_cases()
{
    constexpr IndexT    cid_count = 4;
    std::vector<IndexT> cmts(cid_count * cid_count);
    for(IndexT i = 0; i < cid_count; ++i)
        for(IndexT j = 0; j < cid_count; ++j)
            cmts[i * cid_count + j] = ((i + j) % 3) != 0;
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{cid_count, cid_count});
    d_cmts.view().copy_from(cmts.data());
    CBuffer2DView<IndexT> cmts_view = d_cmts.view();
    auto                  node_pred = RefitNodePred{cmts_view.viewer()};
    auto                  leaf_pred = RefitLeafPred{cmts_view.viewer()};

    struct Chain
    {
        DeviceBuffer<AABB>            aabbs;
        DeviceBuffer<IndexT>          bids;
        DeviceBuffer<IndexT>          cids;
        DeviceBuffer<AABB>            query_aabbs;
        DeviceBuffer<IndexT>          query_bids;
        DeviceBuffer<IndexT>          query_cids;
        InfoStacklessBVH              bvh;
        InfoStacklessBVH::QueryBuffer pairs;
    };
    constexpr std::array<IndexT, 2> tree_sizes{4096, 2816};
    std::array<Chain, 2>            side;
    std::array<Chain, 2>            serial;
    std::array<SideStream, 2>       streams;
    for(auto* chains : {&side, &serial})
        for(auto& chain : *chains)
        {
            chain.pairs.m_pairs.release();
            chain.pairs.reserve(1);
        }

    auto sorted_pairs = [](const InfoStacklessBVH::QueryBuffer& pairs)
    {
        std::vector<Vector2i> result(pairs.size());
        if(!result.empty())
            pairs.view().copy_to(result.data());
        std::ranges::sort(result,
                          [](const Vector2i& l, const Vector2i& r) {
                              return l[0] < r[0] || (l[0] == r[0] && l[1] < r[1]);
                          });
        return result;
    };

    size_t exercised_pair_count = 0;
    for(IndexT step = 0; step < 6; ++step)
    {
        // Steps 0 and 3 rebuild the topology, the others refit it.
        bool refit = step % 3 != 0;
        for(IndexT c = 0; c < 2; ++c)
        {
            auto tree  = lattice_inputs(tree_sizes[c], 3 * step + c);
            auto query = lattice_inputs(tree_sizes[c] / 2, 3 * step + c + 1);
            for(auto* chain : {&side[c], &serial[c]})
            {
                upload_refit(tree, chain->aabbs, chain->bids, chain->cids);
                upload_refit(query, chain->query_aabbs, chain->query_bids, chain->query_cids);
            }
        }

        auto launch = [&](Chain& chain, bool rebuild_query, cudaStream_t stream)
        {
            chain.bvh.launch_query(chain.query_aabbs,
                                   chain.query_bids,
                                   chain.query_cids,
                                   cmts_view,
                                   node_pred,
                                   leaf_pred,
                                   chain.pairs,
                                   rebuild_query,
                                   refit,
                                   stream);
        };
        auto run = [&](std::array<Chain, 2>& chains, bool concurrent)
        {
            for(IndexT c = 0; c < 2; ++c)
            {
                auto& chain = chains[c];
                auto stream = concurrent ? streams[c].fork() : default_stream();
                if(!refit || !chain.bvh.refit(chain.aabbs, chain.bids, chain.cids, stream))
                    chain.bvh.build(chain.aabbs, chain.bids, chain.cids, stream);
                launch(chain, true, stream);
            }
            if(concurrent)
                for(auto& stream : streams)
                    stream.join();
            // Overflow reruns on the default stream, as in the filter.
            for(auto& chain : chains)
            {
                int count = chain.pairs.m_cpNum;
                if(chain.bvh.prepare_query_result(chain.pairs, count))
                    launch(chain, false, default_stream());
            }
        };
        run(side, true);
        run(serial, false);

        for(IndexT c = 0; c < 2; ++c)
        {
            CAPTURE(step, c);
            auto concurrent_pairs = sorted_pairs(side[c].pairs);
            auto serial_pairs     = sorted_pairs(serial[c].pairs);
            REQUIRE(concurrent_pairs.size() == serial_pairs.size());
            REQUIRE(std::memcmp(concurrent_pairs.data(),
                                serial_pairs.data(),
                                serial_pairs.size() * sizeof(Vector2i))
                    == 0);
            exercised_pair_count += serial_pairs.size();
        }
    }
    CHECK(exercised_pair_count > 0);
}

void run_internal_cull_rate_case()
{
    constexpr IndexT    n = 96;
    std::vector<AABB>   aabbs(n);
    std::vector<IndexT> bids(n);
    std::vector<IndexT> cids(n);
    for(IndexT i = 0; i < n; ++i)
    {
        double  x  = static_cast<double>(i % 16) * 5.0e-3;
        double  y  = static_cast<double>(i / 16) * 5.0e-3;
        Vector3 p0 = Vector3{x, y, 0.0};
        Vector3 p1 = Vector3{x + 1.0, y + 1.0, 1.0};
        aabbs[i].extend(p0.cast<float>()).extend(p1.cast<float>());
        bids[i] = static_cast<IndexT>(i % 9);
        cids[i] = static_cast<IndexT>(i % 4);
    }

    DeviceBuffer<AABB> d_aabbs(aabbs.size());
    d_aabbs.view().copy_from(aabbs.data());
    DeviceBuffer<IndexT> d_bids(bids.size());
    d_bids.view().copy_from(bids.data());
    DeviceBuffer<IndexT> d_cids(cids.size());
    d_cids.view().copy_from(cids.data());
    InfoStacklessBVH::Impl impl;
    impl.build(d_aabbs.view(), d_bids.view(), d_cids.view());

    DeviceVar<int>         cp_num;
    DeviceBuffer<int>      node_cull_calls(1);
    DeviceBuffer<int>      node_cull_rejects(1);
    DeviceBuffer<Vector2i> pairs(2048);
    BufferLaunch().fill(cp_num.view(), 0);
    BufferLaunch().fill(node_cull_calls.view(), 0);
    BufferLaunch().fill(node_cull_rejects.view(), 0);

    impl.stacklessSelf(CullRateNodeCull{node_cull_calls.data(),
                                        node_cull_rejects.data()},
                       LeafPredTrue{},
                       cp_num.view(),
                       pairs.view());

    int h_node_cull_calls   = 0;
    int h_node_cull_rejects = 0;
    node_cull_calls.view(0, 1).copy_to(&h_node_cull_calls);
    node_cull_rejects.view(0, 1).copy_to(&h_node_cull_rejects);
    double node_cull_rate = (h_node_cull_calls > 0) ?
                                static_cast<double>(h_node_cull_rejects)
                                    / static_cast<double>(h_node_cull_calls) :
                                0.0;
    fmt::println("internal-cull rate stats: node_cull_calls={}, node_cull_rejects={}, node_cull_rate={:.4f}",
                 h_node_cull_calls,
                 h_node_cull_rejects,
                 node_cull_rate);

    CHECK(h_node_cull_calls > 0);
    CHECK(h_node_cull_rejects > 0);
    CHECK(node_cull_rate > 0.0);
    CHECK(node_cull_rate < 1.0);
}

void run_two_leaf_nodepred_cases()
{
    constexpr IndexT invalid = static_cast<IndexT>(-1);

    auto make_overlapping_aabbs = []()
    {
        std::vector<AABB> aabbs(2);
        Vector3           p00 = Vector3{0.0, 0.0, 0.0};
        Vector3           p01 = Vector3{1.0, 1.0, 1.0};
        Vector3           p10 = Vector3{0.25, 0.25, 0.25};
        Vector3           p11 = Vector3{1.25, 1.25, 1.25};
        aabbs[0].extend(p00.cast<float>()).extend(p01.cast<float>());
        aabbs[1].extend(p10.cast<float>()).extend(p11.cast<float>());
        return aabbs;
    };

    SECTION("two_leaf_bid_only_node_cull")
    {
        // Scenario: both leaves belong to body 0, and self-contact is disabled for body 0.
        // Their leaf CIDs are valid but different, so the internal node CID should be invalid (-1).
        // Expected: NodePred rejects the subtree using BID logic before any leaf-pair output.
        auto                aabbs           = make_overlapping_aabbs();
        std::vector<IndexT> bids            = {0, 0};
        std::vector<IndexT> cids            = {0, 1};
        std::vector<IndexT> is_self_contact = {0};
        std::vector<IndexT> cmts            = {1, 1, 1, 1};
        IndexT self_contact_count = static_cast<IndexT>(is_self_contact.size());
        DeviceBuffer<AABB> d_aabbs(aabbs.size());
        d_aabbs.view().copy_from(aabbs.data());
        DeviceBuffer<IndexT> d_bids(bids.size());
        d_bids.view().copy_from(bids.data());
        DeviceBuffer<IndexT> d_cids(cids.size());
        d_cids.view().copy_from(cids.data());
        DeviceBuffer<IndexT> d_is_self_contact(is_self_contact.size());
        d_is_self_contact.view().copy_from(is_self_contact.data());
        DeviceBuffer2D<IndexT> d_cmts(Extent2D{2, 2});
        d_cmts.view().copy_from(cmts.data());
        DeviceBuffer<int> node_calls(1);
        DeviceBuffer<int> node_rejects(1);
        DeviceBuffer<int> node_invalid_cid_hits(1);
        BufferLaunch().fill(node_calls.view(), 0);
        BufferLaunch().fill(node_rejects.view(), 0);
        BufferLaunch().fill(node_invalid_cid_hits.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(d_cmts.view(),
                   BidOnlyNodeCull{d_bids.viewer(),
                                   d_is_self_contact.viewer(),
                                   self_contact_count,
                                   node_calls.data(),
                                   node_rejects.data(),
                                   node_invalid_cid_hits.data()},
                   LeafPredTrue{},
                   qbuffer);

        int h_calls            = 0;
        int h_rejects          = 0;
        int h_invalid_cid_hits = 0;
        node_calls.view(0, 1).copy_to(&h_calls);
        node_rejects.view(0, 1).copy_to(&h_rejects);
        node_invalid_cid_hits.view(0, 1).copy_to(&h_invalid_cid_hits);
        CHECK(qbuffer.size() == 0);
        CHECK(h_calls > 0);
        CHECK(h_rejects > 0);
        CHECK(h_invalid_cid_hits > 0);
    }

    SECTION("two_leaf_cid_only_node_cull")
    {
        // Scenario: leaves have different bodies (0 and 1), so BID does not cull.
        // Their CIDs are both 1 and cmts(1,1)=0, so NodePred must cull by CID rule.
        auto                aabbs           = make_overlapping_aabbs();
        std::vector<IndexT> bids            = {0, 1};
        std::vector<IndexT> cids            = {1, 1};
        std::vector<IndexT> is_self_contact = {1, 1};
        std::vector<IndexT> cmts            = {1, 1, 1, 0};
        IndexT self_contact_count = static_cast<IndexT>(is_self_contact.size());
        DeviceBuffer<AABB> d_aabbs(aabbs.size());
        d_aabbs.view().copy_from(aabbs.data());
        DeviceBuffer<IndexT> d_bids(bids.size());
        d_bids.view().copy_from(bids.data());
        DeviceBuffer<IndexT> d_cids(cids.size());
        d_cids.view().copy_from(cids.data());
        DeviceBuffer<IndexT> d_is_self_contact(is_self_contact.size());
        d_is_self_contact.view().copy_from(is_self_contact.data());
        DeviceBuffer2D<IndexT> d_cmts(Extent2D{2, 2});
        d_cmts.view().copy_from(cmts.data());
        DeviceBuffer<int> node_rejects(1);
        BufferLaunch().fill(node_rejects.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(d_cmts.view(),
                   CidOnlyNodeCull{d_bids.viewer(),
                                   d_cids.viewer(),
                                   d_cmts.viewer(),
                                   d_is_self_contact.viewer(),
                                   self_contact_count,
                                   node_rejects.data()},
                   LeafPredTrue{},
                   qbuffer);

        int h_rejects = 0;
        node_rejects.view(0, 1).copy_to(&h_rejects);
        CHECK(qbuffer.size() == 0);
        CHECK(h_rejects > 0);
    }

    SECTION("two_leaf_both_invalid_leaf_fallback")
    {
        // Scenario: leaf BIDs/CIDs are legal values but different between the two leaves.
        // This forces internal node BID/CID to become invalid by the merge rule.
        // Expected: NodePred cannot cull, traversal reaches LeafPred fallback.
        auto                aabbs           = make_overlapping_aabbs();
        std::vector<IndexT> bids            = {0, 1};
        std::vector<IndexT> cids            = {0, 1};
        std::vector<IndexT> is_self_contact = {1, 1};
        std::vector<IndexT> cmts            = {1, 1, 1, 1};
        IndexT self_contact_count = static_cast<IndexT>(is_self_contact.size());
        DeviceBuffer<AABB> d_aabbs(aabbs.size());
        d_aabbs.view().copy_from(aabbs.data());
        DeviceBuffer<IndexT> d_bids(bids.size());
        d_bids.view().copy_from(bids.data());
        DeviceBuffer<IndexT> d_cids(cids.size());
        d_cids.view().copy_from(cids.data());
        DeviceBuffer<IndexT> d_is_self_contact(is_self_contact.size());
        d_is_self_contact.view().copy_from(is_self_contact.data());
        DeviceBuffer2D<IndexT> d_cmts(Extent2D{2, 2});
        d_cmts.view().copy_from(cmts.data());
        DeviceBuffer<int> node_rejects(1);
        DeviceBuffer<int> leaf_calls(1);
        DeviceBuffer<int> node_invalid_bid_hits(1);
        DeviceBuffer<int> node_invalid_cid_hits(1);
        BufferLaunch().fill(node_rejects.view(), 0);
        BufferLaunch().fill(leaf_calls.view(), 0);
        BufferLaunch().fill(node_invalid_bid_hits.view(), 0);
        BufferLaunch().fill(node_invalid_cid_hits.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(d_cmts.view(),
                   FallbackNodeCull{d_bids.viewer(),
                                    d_cids.viewer(),
                                    d_cmts.viewer(),
                                    d_is_self_contact.viewer(),
                                    self_contact_count,
                                    node_rejects.data(),
                                    node_invalid_bid_hits.data(),
                                    node_invalid_cid_hits.data()},
                   CountingLeafPredFalse{leaf_calls.data()},
                   qbuffer);

        int h_rejects          = 0;
        int h_leaf_calls       = 0;
        int h_invalid_bid_hits = 0;
        int h_invalid_cid_hits = 0;
        node_rejects.view(0, 1).copy_to(&h_rejects);
        leaf_calls.view(0, 1).copy_to(&h_leaf_calls);
        node_invalid_bid_hits.view(0, 1).copy_to(&h_invalid_bid_hits);
        node_invalid_cid_hits.view(0, 1).copy_to(&h_invalid_cid_hits);
        CHECK(qbuffer.size() == 0);
        CHECK(h_rejects == 0);
        CHECK(h_leaf_calls > 0);
        CHECK(h_invalid_bid_hits > 0);
        CHECK(h_invalid_cid_hits > 0);
    }
}

SimplicialComplex tet()
{
    std::vector<Vector3>  Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    return tetmesh(Vs, Ts);
}

// A selective leaf predicate: most leaves that reach it are rejected, as in
// the trajectory filter, and the accepted ones depend on both pair ids.
struct SelectiveLeafPred
{
    cuda_tool::CDense2D<IndexT> cmts;

    UIPC_GENERIC bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
    {
        if(info.bid_i == info.bid_j || !cmts(info.cid_i, info.cid_j))
            return false;
        return ((info.i * 7 + info.j * 13) % 5) == 0;
    }
};

// Runs an Other query through launch_query and its retry. queries_per_warp
// < 0 selects the per-lane reference traversal.
std::vector<Vector2i> other_query_pairs(InfoStacklessBVH&           bvh,
                                        const DeviceBuffer<AABB>&   query_aabbs,
                                        const DeviceBuffer<IndexT>& query_bids,
                                        const DeviceBuffer<IndexT>& query_cids,
                                        cuda_tool::CBuffer2DView<IndexT> cmts,
                                        int    queries_per_warp,
                                        size_t initial_capacity,
                                        bool&  retried)
{
    bvh.config().warp_cooperative_query = queries_per_warp >= 0;
    bvh.config().queries_per_warp       = std::max(queries_per_warp, 0);
    InfoStacklessBVH::QueryBuffer qbuffer;
    qbuffer.m_pairs.release();
    qbuffer.reserve(initial_capacity);
    auto launch = [&](bool rebuild_query)
    {
        bvh.launch_query(query_aabbs.view(),
                         query_bids.view(),
                         query_cids.view(),
                         cmts,
                         RefitNodePred{cmts.viewer()},
                         SelectiveLeafPred{cmts.viewer()},
                         qbuffer,
                         rebuild_query);
    };
    launch(true);
    int count = qbuffer.m_cpNum;
    retried   = bvh.prepare_query_result(qbuffer, count);
    if(retried)
        launch(false);
    bvh.config() = InfoStacklessBVH::Config{};

    std::vector<Vector2i> pairs(qbuffer.size());
    if(!pairs.empty())
        qbuffer.view().copy_to(pairs.data());
    return pairs;
}

// Random scene with clustered and isolated boxes, so warps mix lanes with no
// leaf hits and lanes with hundreds of them.
void make_clustered_scene(size_t               count,
                          uint32_t             seed,
                          IndexT               cid_count,
                          std::vector<AABB>&   aabbs,
                          std::vector<IndexT>& bids,
                          std::vector<IndexT>& cids)
{
    uint32_t state = seed;
    auto     next  = [&state]()
    {
        state = state * 1664525u + 1013904223u;
        return static_cast<double>(state >> 8) / static_cast<double>(1u << 24);
    };
    aabbs.resize(count);
    bids.resize(count);
    cids.resize(count);
    for(size_t i = 0; i < count; ++i)
    {
        bool    clustered = (i % 4) != 0;
        double  span      = clustered ? 1.0 : 20.0;
        double  radius    = clustered ? 0.05 + 0.1 * next() : 0.02;
        Vector3 c{span * next(), span * next(), span * next()};
        aabbs[i].extend((c - Vector3::Constant(radius)).cast<float>());
        aabbs[i].extend((c + Vector3::Constant(radius)).cast<float>());
        bids[i] = static_cast<IndexT>((i / 3) % 17);
        cids[i] = static_cast<IndexT>(i % cid_count);
    }
}

void run_warp_cooperative_query_cases()
{
    constexpr IndexT    cid_count = 3;
    std::vector<IndexT> cmts(cid_count * cid_count, 1);
    cmts[0 * cid_count + 2] = 0;
    cmts[2 * cid_count + 0] = 0;
    DeviceBuffer2D<IndexT> d_cmts(Extent2D{static_cast<size_t>(cid_count),
                                           static_cast<size_t>(cid_count)});
    d_cmts.view().copy_from(cmts.data());

    for(auto [tree_count, query_count] : {std::pair<size_t, size_t>{2000, 1500},
                                          std::pair<size_t, size_t>{97, 33},
                                          std::pair<size_t, size_t>{1, 70}})
    {
        DYNAMIC_SECTION("tree " << tree_count << " queries " << query_count)
        {
            std::vector<AABB>   tree_aabbs, query_aabbs;
            std::vector<IndexT> tree_bids, tree_cids, query_bids, query_cids;
            make_clustered_scene(tree_count, 7u, cid_count, tree_aabbs, tree_bids, tree_cids);
            make_clustered_scene(query_count, 11u, cid_count, query_aabbs, query_bids, query_cids);

            DeviceBuffer<AABB>   d_tree_aabbs(tree_count);
            DeviceBuffer<IndexT> d_tree_bids(tree_count);
            DeviceBuffer<IndexT> d_tree_cids(tree_count);
            d_tree_aabbs.view().copy_from(tree_aabbs.data());
            d_tree_bids.view().copy_from(tree_bids.data());
            d_tree_cids.view().copy_from(tree_cids.data());
            DeviceBuffer<AABB>   d_query_aabbs(query_count);
            DeviceBuffer<IndexT> d_query_bids(query_count);
            DeviceBuffer<IndexT> d_query_cids(query_count);
            d_query_aabbs.view().copy_from(query_aabbs.data());
            d_query_bids.view().copy_from(query_bids.data());
            d_query_cids.view().copy_from(query_cids.data());

            InfoStacklessBVH bvh;
            bvh.build(d_tree_aabbs.view(), d_tree_bids.view(), d_tree_cids.view());

            // Brute force over the same node and leaf predicates.
            std::vector<Vector2i> expected;
            for(IndexT i = 0; i < static_cast<IndexT>(query_count); ++i)
                for(IndexT j = 0; j < static_cast<IndexT>(tree_count); ++j)
                {
                    if(!query_aabbs[i].intersects(tree_aabbs[j]))
                        continue;
                    if(query_bids[i] == tree_bids[j]
                       || !allow_contact(cmts, cid_count, query_cids[i], tree_cids[j]))
                        continue;
                    if(((i * 7 + j * 13) % 5) == 0)
                        expected.emplace_back(i, j);
                }

            for(size_t capacity : {size_t{1}, size_t{1} << 20})
            {
                bool retried_lane = false;
                auto per_lane     = other_query_pairs(
                    bvh, d_query_aabbs, d_query_bids, d_query_cids, d_cmts.view(), -1, capacity, retried_lane);
                CAPTURE(capacity, expected.size());
                CHECK(retried_lane == (expected.size() > capacity));
                check_cp_exact(per_lane, expected);
                for(int queries_per_warp : {0, 1, 2, 5, 32})
                {
                    CAPTURE(queries_per_warp);
                    bool retried = false;
                    auto shared  = other_query_pairs(bvh,
                                                    d_query_aabbs,
                                                    d_query_bids,
                                                    d_query_cids,
                                                    d_cmts.view(),
                                                    queries_per_warp,
                                                    capacity,
                                                    retried);
                    CHECK(retried == retried_lane);
                    check_cp_exact(shared, per_lane);
                }
            }
        }
    }
}
}  // namespace test_info_stackless_bvh

TEST_CASE("info_stackless_bvh", "[collision detection]")
{
    using namespace test_info_stackless_bvh;

    SECTION("tet")
    {
        fmt::println("tet:");
        run_info_stackless_bvh_test(tet());
    }

    SECTION("cube.obj")
    {
        fmt::println("cube.obj:");
        SimplicialComplexIO io;
        auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
        run_info_stackless_bvh_test(mesh);
    }

    SECTION("internal_cull_proof")
    {
        fmt::println("internal_cull_proof:");
        run_internal_cull_proof_case();
    }

    SECTION("internal_cull_rate")
    {
        fmt::println("internal_cull_rate:");
        run_internal_cull_rate_case();
    }

    SECTION("two_leaf_nodepred_cases")
    {
        fmt::println("two_leaf_nodepred_cases:");
        run_two_leaf_nodepred_cases();
    }
}

TEST_CASE("info_stackless_bvh self Morton-rank pruning", "[collision detection][self_rank_pruning]")
{
    using namespace test_info_stackless_bvh;
    run_internal_cull_proof_case();
    run_self_rank_reordered_node_case();
}

TEST_CASE("info_stackless_bvh CTA and queue boundaries", "[collision detection][bvh_cta_sweep]")
{
    using namespace test_info_stackless_bvh;
    fmt::println("Self CTA={} queue={}; Other CTA={} queue={}",
                 uipc::info_stackless_detail::K_SELF_THREADS,
                 uipc::info_stackless_detail::K_SELF_MAX_RES_PER_BLOCK,
                 uipc::info_stackless_detail::K_OTHER_THREADS,
                 uipc::info_stackless_detail::K_OTHER_MAX_RES_PER_BLOCK);
    run_cta_queue_boundary_cases();
}

TEST_CASE("info_stackless_bvh topology refit", "[collision detection][bvh_refit][line_search]")
{
    using namespace test_info_stackless_bvh;
    run_refit_cases();
}

TEST_CASE("info_stackless_bvh side streams", "[collision detection][bvh_side_stream]")
{
    using namespace test_info_stackless_bvh;
    run_side_stream_cases();
}

TEST_CASE("info_stackless_bvh query Morton order reuse",
          "[collision detection][bvh_query_order][line_search]")
{
    using namespace test_info_stackless_bvh;
    run_query_order_reuse_cases();
}

TEST_CASE("info_stackless_bvh warp-cooperative query", "[collision detection][bvh_warp_query]")
{
    using namespace test_info_stackless_bvh;
    run_warp_cooperative_query_cases();
}
