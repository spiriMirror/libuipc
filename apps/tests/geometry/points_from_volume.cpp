#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;


TEST_CASE("points_from_volume", "[points]")
{
    SimplicialComplexIO io;
    auto                output_path = AssetDir::output_path(__FILE__);

    SECTION("cube")
    {
        auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
        auto pc = points_from_volume(mesh, 0.1);
        io.write(fmt::format("{}/points_from_volume.cube.obj", output_path), pc);
    }

    SECTION("bunny0")
    {
        auto mesh = io.read_msh(fmt::format("{}bunny0.msh", AssetDir::tetmesh_path()));
        auto pc = points_from_volume(mesh, 0.03);
        io.write(fmt::format("{}/points_from_volume.bunny0.obj", output_path), pc);
    }
}
