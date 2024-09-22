#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("label_surface", "[surface]")
{
    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    label_surface(mesh);

    auto v_is_surf_view = mesh.vertices().find<IndexT>(builtin::is_surf)->view();
    auto e_is_surf_view = mesh.edges().find<IndexT>(builtin::is_surf)->view();
    auto f_is_surf_view = mesh.triangles().find<IndexT>(builtin::is_surf)->view();
    // for a cube, all vertices are on the surface
    REQUIRE(std::ranges::all_of(v_is_surf_view, [](auto s) -> bool { return s; }));
    // for a cube, all edges are on the surface
    REQUIRE(std::ranges::all_of(e_is_surf_view, [](auto s) -> bool { return s; }));

    // for a cube, there are some triangles are on the surface, and others are not
    REQUIRE(std::ranges::any_of(f_is_surf_view, [](auto s) -> bool { return s; }));
    REQUIRE(std::ranges::any_of(f_is_surf_view, [](auto s) -> bool { return !s; }));
}
