#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <format>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("instancing", "[geometry]")
{
    SimplicialComplexIO io;

    auto mesh = io.read(std::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto Is = mesh.instances();

    REQUIRE(Is.size() == 1);  // the initial mesh is an instance of itself

    Is.resize(5);

    auto trans      = Is.find<Matrix4x4>("transform");
    auto trans_view = trans->view();
    for(auto&& t : trans_view)
    {
        REQUIRE(t == Matrix4x4::Identity());
    }
}