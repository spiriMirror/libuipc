#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("tetrahedralize", "[tetrahedralize]")
{
    SimplicialComplexIO io;
    auto cube = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    auto output_path = AssetDir::output_path(__FILE__);

    auto tet_cube = tetrahedralize(cube);
}
