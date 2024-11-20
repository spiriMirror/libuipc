#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;
TEST_CASE("read_msh", "[io]")
{
    SimplicialComplexIO io;

    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.tetrahedra().size() == 5);
    REQUIRE(mesh.dim() == 3);

    REQUIRE_THROWS_AS(io.read_msh(fmt::format("{}NOMESH.msh", AssetDir::tetmesh_path())),
                      GeometryIOError);
}

TEST_CASE("read_obj", "[io]")
{
    SimplicialComplexIO io;
    auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.triangles().size() == 12);
    REQUIRE(mesh.tetrahedra().size() == 0);
    REQUIRE(mesh.dim() == 2);

    REQUIRE_THROWS_AS(io.read_obj(fmt::format("{}NOMESH.obj", AssetDir::trimesh_path())),
                      GeometryIOError);
}

TEST_CASE("read_ply", "[io]")
{
    SimplicialComplexIO io;
    auto mesh = io.read_ply(fmt::format("{}cube.ply", AssetDir::trimesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.triangles().size() == 12);
    REQUIRE(mesh.tetrahedra().size() == 0);
    REQUIRE(mesh.dim() == 2);

    REQUIRE_THROWS_AS(io.read_ply(fmt::format("{}NOMESH.ply", AssetDir::trimesh_path())),
                      GeometryIOError);
}

TEST_CASE("write_msh", "[io]")
{
    auto output_path = AssetDir::output_path(__FILE__);

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    io.write_msh(fmt::format("{}cube_out.msh", output_path), mesh);
    auto mesh_out = io.read_msh(fmt::format("{}cube_out.msh", output_path));
    REQUIRE(mesh_out.vertices().size() == 8);
    REQUIRE(mesh_out.tetrahedra().size() == 5);
    REQUIRE(mesh_out.dim() == 3);
}

TEST_CASE("write_obj", "[io]")
{
    auto output_path = AssetDir::output_path(__FILE__);

    SimplicialComplexIO io;
    auto mesh = io.read_obj(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    io.write_obj(fmt::format("{}cube_out.obj", output_path), mesh);
    auto mesh_out = io.read_obj(fmt::format("{}cube_out.obj", output_path));
    REQUIRE(mesh_out.vertices().size() == 8);
    REQUIRE(mesh_out.triangles().size() == 12);
    REQUIRE(mesh_out.tetrahedra().size() == 0);
    REQUIRE(mesh_out.dim() == 2);
}