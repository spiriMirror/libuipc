#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("closest_vertex_triangle_pairs", "[geometry][closest_vertex_triangle_pairs]")
{
    // Vertex mesh: two points
    vector<Vector3> vert_pos = {
        {0.0, 0.0, 0.5},   // above triangle
        {0.5, 0.5, 0.0},   // on triangle
    };
    auto vertex_mesh = pointcloud(vert_pos);

    // Triangle mesh: one triangle in z=0
    vector<Vector3> tri_pos = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
    };
    vector<Vector3i> tri_faces = {{0, 1, 2}};
    auto triangle_mesh = trimesh(tri_pos, tri_faces);

    SECTION("max_distance only")
    {
        Float max_d = 1.0;
        auto pairs  = closest_vertex_triangle_pairs(vertex_mesh, triangle_mesh, max_d);

        REQUIRE(pairs.instances().size() == 2);
        auto topo_slot = pairs.instances().find<Vector2i>(builtin::topo);
        REQUIRE(topo_slot);
        auto topo_view = topo_slot->view();
        REQUIRE(topo_view.size() == 2);
        // (vertex_index, triangle_index)
        REQUIRE((topo_view[0](0) == 0 && topo_view[0](1) == 0));
        REQUIRE((topo_view[1](0) == 1 && topo_view[1](1) == 0));
    }

    SECTION("small max_distance yields fewer pairs")
    {
        Float max_d = 0.2;
        auto pairs  = closest_vertex_triangle_pairs(vertex_mesh, triangle_mesh, max_d);
        // Only (0.5, 0.5, 0) is close; (0,0,0.5) is 0.5 away
        REQUIRE(pairs.instances().size() == 1);
        auto topo_slot = pairs.instances().find<Vector2i>(builtin::topo);
        REQUIRE(topo_slot);
        REQUIRE(topo_slot->view()[0](0) == 1);
        REQUIRE(topo_slot->view()[0](1) == 0);
    }

    SECTION("zero max_distance yields no pairs")
    {
        auto pairs = closest_vertex_triangle_pairs(vertex_mesh, triangle_mesh, 0.0);
        REQUIRE(pairs.instances().size() == 0);
    }
}
