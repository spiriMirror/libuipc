#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/mesh_partition.h>

namespace
{
using namespace uipc;
using namespace uipc::geometry;

SimplicialComplex make_chain()
{
    vector<Vector3>  vertices;
    vector<Vector2i> edges;
    constexpr IndexT vertex_count = 12;

    vertices.reserve(vertex_count);
    edges.reserve(vertex_count - 1);
    for(IndexT i = 0; i < vertex_count; ++i)
    {
        vertices.emplace_back(static_cast<Float>(i), 0.0, 0.0);
        if(i > 0)
            edges.emplace_back(i - 1, i);
    }
    return linemesh(vertices, edges);
}

vector<IndexT> partition_ids(const SimplicialComplex& mesh)
{
    const auto partition = mesh.vertices().find<IndexT>("mesh_part");
    REQUIRE(partition);
    const auto partition_view = partition->view();
    return {partition_view.begin(), partition_view.end()};
}
}  // namespace

TEST_CASE("mesh_partition uses the embedded METIS implementation", "[mesh_partition][metis]")
{
    constexpr SizeT max_partition_size = 3;
    auto            first              = make_chain();
    auto            second             = make_chain();

    mesh_partition(first, max_partition_size);
    mesh_partition(second, max_partition_size);

    const auto first_ids  = partition_ids(first);
    const auto second_ids = partition_ids(second);
    REQUIRE(first_ids == second_ids);

    const IndexT max_id = *std::ranges::max_element(first_ids);
    REQUIRE(max_id >= 1);
    vector<SizeT> sizes(static_cast<SizeT>(max_id + 1), 0);
    for(const IndexT id : first_ids)
    {
        REQUIRE(id >= 0);
        REQUIRE(id <= max_id);
        ++sizes[static_cast<SizeT>(id)];
    }
    for(const SizeT size : sizes)
    {
        REQUIRE(size > 0);
        REQUIRE(size <= max_partition_size);
    }
}

TEST_CASE("mesh_partition rejects a zero partition size", "[mesh_partition][metis]")
{
    auto mesh = make_chain();
    REQUIRE_THROWS_AS(mesh_partition(mesh, 0), Exception);
}
