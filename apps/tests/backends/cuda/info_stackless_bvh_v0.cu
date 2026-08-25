#include <type_define.h>
#include <app/app.h>
#include <collision_detection/info_stackless_bvh_v0.h>
#include <algorithm>
#include <utility>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
struct KeepAllV0Nodes
{
    UIPC_GENERIC bool operator()(const InfoStacklessBVHV0::NodePredInfo&) const
    {
        return true;
    }
};

struct KeepAllV0Leaves
{
    UIPC_GENERIC bool operator()(IndexT, IndexT) const { return true; }
};

std::vector<AABB> make_boxes(IndexT count, Float phase)
{
    std::vector<AABB> boxes(count);
    for(IndexT i = 0; i < count; ++i)
    {
        Float x = static_cast<Float>(i % 17) * 0.17 + phase * 0.03;
        Float y = static_cast<Float>((i / 17) % 17) * 0.19 + phase * 0.02;
        Float z = static_cast<Float>(i % 5) * 0.03;
        boxes[i]
            .extend(Vector3{x, y, z}.cast<float>())
            .extend(Vector3{x + 0.22, y + 0.23, z + 0.21}.cast<float>());
    }
    return boxes;
}

using Pair = std::pair<IndexT, IndexT>;

std::vector<Pair> copy_pairs(const InfoStacklessBVHV0::QueryBuffer& query_buffer)
{
    std::vector<Vector2i> device_pairs(query_buffer.size());
    query_buffer.view().copy_to(device_pairs.data());

    std::vector<Pair> pairs;
    pairs.reserve(device_pairs.size());
    for(const auto& pair : device_pairs)
        pairs.emplace_back(pair.x(), pair.y());
    std::ranges::sort(pairs);
    return pairs;
}

std::vector<Pair> brute_force_detect(const std::vector<AABB>& boxes)
{
    std::vector<Pair> pairs;
    for(IndexT i = 0; i < static_cast<IndexT>(boxes.size()); ++i)
    {
        for(IndexT j = i + 1; j < static_cast<IndexT>(boxes.size()); ++j)
        {
            if(boxes[i].intersects(boxes[j]))
                pairs.emplace_back(i, j);
        }
    }
    return pairs;
}

std::vector<Pair> brute_force_query(const std::vector<AABB>& query_boxes,
                                    const std::vector<AABB>& tree_boxes)
{
    std::vector<Pair> pairs;
    for(IndexT i = 0; i < static_cast<IndexT>(query_boxes.size()); ++i)
    {
        for(IndexT j = 0; j < static_cast<IndexT>(tree_boxes.size()); ++j)
        {
            if(query_boxes[i].intersects(tree_boxes[j]))
                pairs.emplace_back(i, j);
        }
    }
    return pairs;
}

void run_v0_parity_case(InfoStacklessBVHV0& bvh, IndexT count, Float phase)
{
    auto boxes       = make_boxes(count, phase);
    auto query_boxes = make_boxes(count / 2 + 1, phase + 0.5);

    std::vector<IndexT> bids(count);
    std::vector<IndexT> cids(count);
    for(IndexT i = 0; i < count; ++i)
    {
        bids[i] = i % 7;
        cids[i] = i % 3;
    }

    std::vector<IndexT> query_bids(query_boxes.size());
    std::vector<IndexT> query_cids(query_boxes.size());
    for(IndexT i = 0; i < static_cast<IndexT>(query_boxes.size()); ++i)
    {
        query_bids[i] = (i + 2) % 7;
        query_cids[i] = (i + 1) % 3;
    }

    cuda_tool::DeviceBuffer<AABB> d_boxes(boxes.size());
    d_boxes.view().copy_from(boxes.data());
    cuda_tool::DeviceBuffer<IndexT> d_bids(bids.size());
    d_bids.view().copy_from(bids.data());
    cuda_tool::DeviceBuffer<IndexT> d_cids(cids.size());
    d_cids.view().copy_from(cids.data());

    cuda_tool::DeviceBuffer<AABB> d_query_boxes(query_boxes.size());
    d_query_boxes.view().copy_from(query_boxes.data());
    cuda_tool::DeviceBuffer<IndexT> d_query_bids(query_bids.size());
    d_query_bids.view().copy_from(query_bids.data());
    cuda_tool::DeviceBuffer<IndexT> d_query_cids(query_cids.size());
    d_query_cids.view().copy_from(query_cids.data());

    std::vector<IndexT> contact_matrix(9, 1);
    cuda_tool::DeviceBuffer2D<IndexT> d_contact_matrix(cuda_tool::Extent2D{3, 3});
    d_contact_matrix.view().copy_from(contact_matrix.data());

    bvh.build(d_boxes.view(), d_bids.view(), d_cids.view());
    InfoStacklessBVHV0::QueryBuffer query_buffer;

    bvh.detect(d_contact_matrix.view(), KeepAllV0Nodes{}, KeepAllV0Leaves{}, query_buffer);
    CHECK(copy_pairs(query_buffer) == brute_force_detect(boxes));

    bvh.query(d_query_boxes.view(),
              d_query_bids.view(),
              d_query_cids.view(),
              d_contact_matrix.view(),
              KeepAllV0Nodes{},
              KeepAllV0Leaves{},
              query_buffer);
    CHECK(copy_pairs(query_buffer) == brute_force_query(query_boxes, boxes));
}
}  // namespace

TEST_CASE("info_stackless_bvh_v0", "[collision detection]")
{
    InfoStacklessBVHV0 bvh;
    run_v0_parity_case(bvh, 96, 0.0);
    run_v0_parity_case(bvh, 257, 1.0);
}
