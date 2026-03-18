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
    MUDA_GENERIC bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
    {
        return ((info.query_id + info.node_bid + info.node_cid) % 5) != 0;
    }
};

struct LeafPred
{
    MUDA_GENERIC bool operator()(IndexT i, IndexT j) const { return ((i ^ j) & 1) == 0; }
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
    NodePred              np;
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
            if(!np({i, bids[j], cids[j]}))
                continue;
            if(!lp(i, j))
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
    NodePred              np;
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
            if(!np({i, tree_bids[j], tree_cids[j]}))
                continue;
            if(!lp(i, j))
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
}
