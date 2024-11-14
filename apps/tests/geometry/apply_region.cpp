#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("apply_region", "[connected_components]")
{
    SimplicialComplexIO io;
    auto cube = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    auto output_path = AssetDir::output_path(__FILE__);

    {
        SimplicialComplex mesh = cube;

        label_connected_vertices(mesh);
        auto region      = mesh.vertices().find<IndexT>("region");
        auto region_view = region->view();

        // for a cube, all vertices are connected
        REQUIRE(std::ranges::all_of(region_view,
                                    [](auto r) -> bool { return r == 0; }));
    }

    {
        SimplicialComplex mesh1 = cube;
        SimplicialComplex mesh2 = cube;
        auto              Ps    = view(mesh2.positions());
        std::ranges::transform(Ps,
                               Ps.begin(),
                               [](const auto& p) {
                                   return p + Vector3{0, 1, 0};
                               });

        auto merged_mesh = merge({&mesh1, &mesh2});
        label_connected_vertices(merged_mesh);
        label_region(merged_mesh);

        auto vert_region      = merged_mesh.vertices().find<IndexT>("region");
        auto vert_region_view = vert_region->view();

        auto edge_region      = merged_mesh.edges().find<IndexT>("region");
        auto edge_region_view = edge_region->view();

        auto tri_region      = merged_mesh.triangles().find<IndexT>("region");
        auto tri_region_view = tri_region->view();

        auto check_region = [&](span<const IndexT> region_view)
        {
            auto count = region_view.size();
            auto half  = count / 2;

            auto first_half  = region_view.subspan(0, half);
            auto second_half = region_view.subspan(half, half);

            // first half has the same region
            REQUIRE(std::ranges::adjacent_find(first_half, std::not_equal_to<IndexT>())
                    == first_half.end());

            // second half has the same region
            REQUIRE(std::ranges::adjacent_find(second_half, std::not_equal_to<IndexT>())
                    == second_half.end());

            // first half and second half have different regions
            REQUIRE(first_half.front() != second_half.front());
        };

        check_region(vert_region_view);
        check_region(edge_region_view);
        check_region(tri_region_view);

        SpreadSheetIO sio{output_path};
        sio.write_json("merged_mesh", merged_mesh);

        auto apart_meshes = apply_region(merged_mesh);

        REQUIRE(apart_meshes.size() == 2);

        for(auto&& [i, apart_mesh] : enumerate(apart_meshes))
        {
            auto region_count = apart_mesh.meta().find<IndexT>("region_count");
            REQUIRE(!region_count);  // the region_count attribute should be removed

            auto vert_region = apart_mesh.vertices().find<IndexT>("region");

            REQUIRE(!vert_region);  // the region attribute should be removed

            auto edge_region = apart_mesh.edges().find<IndexT>("region");

            REQUIRE(!edge_region);  // the region attribute should be removed

            auto tri_region = apart_mesh.triangles().find<IndexT>("region");

            REQUIRE(!tri_region);  // the region attribute should be removed

            REQUIRE(apart_mesh.vertices().find<Vector3>(builtin::position));  // the position attribute should be kept

            REQUIRE(apart_mesh.vertices().size() == cube.vertices().size());

            REQUIRE(apart_mesh.edges().size() == cube.edges().size());

            REQUIRE(apart_mesh.triangles().size() == cube.triangles().size());
        }

        io.write(fmt::format("{}merged_mesh.obj", output_path), merged_mesh);
        io.write(fmt::format("{}apart_mesh_0.obj", output_path), apart_meshes[0]);
        io.write(fmt::format("{}apart_mesh_1.obj", output_path), apart_meshes[1]);
    }
}
