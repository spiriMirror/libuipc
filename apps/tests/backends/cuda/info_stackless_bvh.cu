#include <type_define.h>
#include <app/app.h>
#include <app/asset_dir.h>
#include <collision_detection/info_stackless_bvh.h>
#include <uipc/geometry.h>
#include <uipc/uipc.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>
#include <algorithm>
#include <iterator>
#include <list>

using namespace muda;
using namespace uipc;
using namespace uipc::geometry;
using namespace uipc::backend::cuda;

namespace test_info_stackless_bvh
{
struct NodePred
{
    MUDA_GENERIC bool operator()(const InfoStacklessBVH::NodePredInfo&) const { return true; }
};

struct LeafPred
{
    MUDA_GENERIC bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
    {
        return ((info.i ^ info.j) & 1) == 0;
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
            if(!lp(InfoStacklessBVH::LeafPredInfo{i, j, query_bids[i], query_cids[i], tree_bids[j], tree_cids[j]}))
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

    constexpr IndexT cid_count = 4;
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
    constexpr IndexT n = 64;
    std::vector<AABB>   aabbs(n);
    std::vector<IndexT> bids(n, 3);
    std::vector<IndexT> cids(n, 0);
    for(IndexT i = 0; i < n; ++i)
    {
        double x  = static_cast<double>(i) * 1.0e-3;
        Vector3 p0 = Vector3{x, x, x};
        Vector3 p1 = Vector3{x + 1.0, x + 1.0, x + 1.0};
        AABB box;
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

    DeviceVar<int>      cp_num;
    DeviceBuffer<int>   node_cull_calls(1);
    DeviceBuffer<int>   leaf_pair_calls(1);
    DeviceBuffer<Vector2i> pairs(16);
    BufferLaunch().fill(cp_num.view(), 0);
    BufferLaunch().fill(node_cull_calls.view(), 0);
    BufferLaunch().fill(leaf_pair_calls.view(), 0);

    impl.stacklessSelf(
        [calls = node_cull_calls.data()] __device__(const InfoStacklessBVH::NodePredInfo&)
        {
            atomicAdd(calls, 1);
            return false;
        },
        [leaf_calls = leaf_pair_calls.data()] __device__(const InfoStacklessBVH::LeafPredInfo&)
        {
            atomicAdd(leaf_calls, 1);
            return true;
        },
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

    CHECK(h_node_cull_calls == n);
    CHECK(h_leaf_pair_calls == 0);
    CHECK(h_pairs == 0);
}

void run_internal_cull_rate_case()
{
    constexpr IndexT n = 96;
    std::vector<AABB>   aabbs(n);
    std::vector<IndexT> bids(n);
    std::vector<IndexT> cids(n);
    for(IndexT i = 0; i < n; ++i)
    {
        double x = static_cast<double>(i % 16) * 5.0e-3;
        double y = static_cast<double>(i / 16) * 5.0e-3;
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

    DeviceVar<int>      cp_num;
    DeviceBuffer<int>   node_cull_calls(1);
    DeviceBuffer<int>   node_cull_rejects(1);
    DeviceBuffer<Vector2i> pairs(2048);
    BufferLaunch().fill(cp_num.view(), 0);
    BufferLaunch().fill(node_cull_calls.view(), 0);
    BufferLaunch().fill(node_cull_rejects.view(), 0);

    impl.stacklessSelf(
        [calls = node_cull_calls.data(),
         rejects = node_cull_rejects.data()] __device__(const InfoStacklessBVH::NodePredInfo& info)
        {
            atomicAdd(calls, 1);
            bool keep = (info.query_id % 3) != 0;
            if(!keep)
                atomicAdd(rejects, 1);
            return keep;
        },
        [] __device__(const InfoStacklessBVH::LeafPredInfo&)
        {
            return true;
        },
        cp_num.view(),
        pairs.view());

    int h_node_cull_calls = 0;
    int h_node_cull_rejects = 0;
    node_cull_calls.view(0, 1).copy_to(&h_node_cull_calls);
    node_cull_rejects.view(0, 1).copy_to(&h_node_cull_rejects);
    double node_cull_rate = (h_node_cull_calls > 0) ?
                                static_cast<double>(h_node_cull_rejects) /
                                    static_cast<double>(h_node_cull_calls) :
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
        Vector3 p00 = Vector3{0.0, 0.0, 0.0};
        Vector3 p01 = Vector3{1.0, 1.0, 1.0};
        Vector3 p10 = Vector3{0.25, 0.25, 0.25};
        Vector3 p11 = Vector3{1.25, 1.25, 1.25};
        aabbs[0].extend(p00.cast<float>()).extend(p01.cast<float>());
        aabbs[1].extend(p10.cast<float>()).extend(p11.cast<float>());
        return aabbs;
    };

    SECTION("two_leaf_bid_only_node_cull")
    {
        // Scenario: both leaves belong to body 0, and self-contact is disabled for body 0.
        // Their leaf CIDs are valid but different, so the internal node CID should be invalid (-1).
        // Expected: NodePred rejects the subtree using BID logic before any leaf-pair output.
        auto aabbs = make_overlapping_aabbs();
        std::vector<IndexT> bids = {0, 0};
        std::vector<IndexT> cids = {0, 1};
        std::vector<IndexT> is_self_contact = {0};
        std::vector<IndexT> cmts = {1, 1, 1, 1};
        IndexT              self_contact_count = static_cast<IndexT>(is_self_contact.size());
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
        DeviceBuffer<int>   node_calls(1);
        DeviceBuffer<int>   node_rejects(1);
        DeviceBuffer<int>   node_invalid_cid_hits(1);
        BufferLaunch().fill(node_calls.view(), 0);
        BufferLaunch().fill(node_rejects.view(), 0);
        BufferLaunch().fill(node_invalid_cid_hits.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(
            d_cmts.view(),
            [bids = d_bids.viewer().name("bids"),
             is_self_contact = d_is_self_contact.viewer().name("is_self_contact"),
             self_contact_count,
             calls = node_calls.data(),
             rejects = node_rejects.data(),
             invalid_cid_hits = node_invalid_cid_hits.data()] __device__(const InfoStacklessBVH::NodePredInfo& info)
            {
                atomicAdd(calls, 1);
                if(info.node_cid == invalid)
                    atomicAdd(invalid_cid_hits, 1);
                auto qbid = bids(info.query_id);
                bool self_contact_disabled =
                    (qbid != invalid) && (qbid < self_contact_count) && !is_self_contact(qbid);
                bool keep = !(info.node_bid != invalid && qbid != invalid && info.node_bid == qbid
                              && self_contact_disabled);
                if(!keep)
                    atomicAdd(rejects, 1);
                return keep;
            },
            [] __device__(InfoStacklessBVH::LeafPredInfo) { return true; },
            qbuffer);

        int h_calls = 0;
        int h_rejects = 0;
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
        auto aabbs = make_overlapping_aabbs();
        std::vector<IndexT> bids = {0, 1};
        std::vector<IndexT> cids = {1, 1};
        std::vector<IndexT> is_self_contact = {1, 1};
        std::vector<IndexT> cmts = {1, 1, 1, 0};
        IndexT              self_contact_count = static_cast<IndexT>(is_self_contact.size());
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
        DeviceBuffer<int>   node_rejects(1);
        BufferLaunch().fill(node_rejects.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(
            d_cmts.view(),
            [bids = d_bids.viewer().name("bids"),
             cids = d_cids.viewer().name("cids"),
             cmts = d_cmts.viewer().name("cmts"),
             is_self_contact = d_is_self_contact.viewer().name("is_self_contact"),
             self_contact_count,
             rejects = node_rejects.data()] __device__(const InfoStacklessBVH::NodePredInfo& info)
            {
                auto qbid = bids(info.query_id);
                auto qcid = cids(info.query_id);
                bool bid_cull = info.node_bid != invalid && qbid != invalid && info.node_bid == qbid
                                && (qbid < self_contact_count) && !is_self_contact(qbid);
                bool cid_cull = info.node_cid != invalid && qcid != invalid && !cmts(qcid, info.node_cid);
                bool keep = !(bid_cull || cid_cull);
                if(!keep)
                    atomicAdd(rejects, 1);
                return keep;
            },
            [] __device__(InfoStacklessBVH::LeafPredInfo) { return true; },
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
        auto aabbs = make_overlapping_aabbs();
        std::vector<IndexT> bids = {0, 1};
        std::vector<IndexT> cids = {0, 1};
        std::vector<IndexT> is_self_contact = {1, 1};
        std::vector<IndexT> cmts = {1, 1, 1, 1};
        IndexT              self_contact_count = static_cast<IndexT>(is_self_contact.size());
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
        DeviceBuffer<int>   node_rejects(1);
        DeviceBuffer<int>   leaf_calls(1);
        DeviceBuffer<int>   node_invalid_bid_hits(1);
        DeviceBuffer<int>   node_invalid_cid_hits(1);
        BufferLaunch().fill(node_rejects.view(), 0);
        BufferLaunch().fill(leaf_calls.view(), 0);
        BufferLaunch().fill(node_invalid_bid_hits.view(), 0);
        BufferLaunch().fill(node_invalid_cid_hits.view(), 0);

        InfoStacklessBVH bvh;
        bvh.build(d_aabbs.view(), d_bids.view(), d_cids.view());
        InfoStacklessBVH::QueryBuffer qbuffer;
        qbuffer.reserve(8);
        bvh.detect(
            d_cmts.view(),
            [bids = d_bids.viewer().name("bids"),
             cids = d_cids.viewer().name("cids"),
             cmts = d_cmts.viewer().name("cmts"),
             is_self_contact = d_is_self_contact.viewer().name("is_self_contact"),
             self_contact_count,
             rejects = node_rejects.data(),
             invalid_bid_hits = node_invalid_bid_hits.data(),
             invalid_cid_hits = node_invalid_cid_hits.data()] __device__(const InfoStacklessBVH::NodePredInfo& info)
            {
                if(info.node_bid == invalid)
                    atomicAdd(invalid_bid_hits, 1);
                if(info.node_cid == invalid)
                    atomicAdd(invalid_cid_hits, 1);
                auto qbid = bids(info.query_id);
                auto qcid = cids(info.query_id);
                bool bid_cull = info.node_bid != invalid && qbid != invalid && info.node_bid == qbid
                                && (qbid < self_contact_count) && !is_self_contact(qbid);
                bool cid_cull = info.node_cid != invalid && qcid != invalid && !cmts(qcid, info.node_cid);
                bool keep = !(bid_cull || cid_cull);
                if(!keep)
                    atomicAdd(rejects, 1);
                return keep;
            },
            [leaf_calls = leaf_calls.data()] __device__(InfoStacklessBVH::LeafPredInfo)
            {
                atomicAdd(leaf_calls, 1);
                return false;
            },
            qbuffer);

        int h_rejects = 0;
        int h_leaf_calls = 0;
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
