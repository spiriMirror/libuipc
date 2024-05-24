#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/geometry/io.h>
#include <format>

using namespace uipc;
using namespace uipc::geometry;
TEST_CASE("read_msh", "[geometry]")
{
    auto mesh = read_msh(std::format("{}cube.msh", AssetDir::tetmesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.edges().size() == 0);
    REQUIRE(mesh.triangles().size() == 0);
    REQUIRE(mesh.tetrahedra().size() == 5);
    REQUIRE(mesh.dim() == 3);

    REQUIRE_THROWS_AS(read_msh(std::format("{}NOMESH.msh", AssetDir::tetmesh_path())),
                      GeometryIOError);
}

TEST_CASE("read_obj", "[geometry]")
{
    auto mesh = read_obj(std::format("{}cube.obj", AssetDir::trimesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.edges().size() == 0);
    REQUIRE(mesh.triangles().size() == 12);
    REQUIRE(mesh.tetrahedra().size() == 0);
    REQUIRE(mesh.dim() == 2);

    REQUIRE_THROWS_AS(read_obj(std::format("{}NOMESH.obj", AssetDir::trimesh_path())),
                      GeometryIOError);
}
