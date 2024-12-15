#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("instancing", "[instance]")
{
    SimplicialComplexIO io;

    auto mesh = io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    REQUIRE(mesh.instances().size() == 1);  // the initial mesh is an instance of itself

    mesh.instances().resize(5);

    auto trans      = mesh.instances().find<Matrix4x4>(builtin::transform);
    auto trans_view = trans->view();
    for(auto&& t : trans_view)
    {
        REQUIRE(t == Matrix4x4::Identity());
    }
}