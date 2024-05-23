#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/geometry/io.h>
#include <format>

using namespace uipc;
using namespace uipc::geometry;
TEST_CASE("readmesh", "[geometry]")
{
    auto mesh = read_msh(std::format("{}cube.msh", AssetDir::tetmesh_path()));
    REQUIRE(mesh.vertices().size() == 8);
    REQUIRE(mesh.tetrahedra().size() == 5);

    REQUIRE_THROWS_AS(read_msh(std::format("{}NOMESH.msh", AssetDir::tetmesh_path())),
                      GeometryIOError);
}
