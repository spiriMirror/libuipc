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

    auto tet_cube = tetrahedralize(cube, 0.1);
    label_surface(tet_cube);
    label_triangle_orient(tet_cube);
    auto out  = flip_inward_triangles(tet_cube);
    auto surf = extract_surface(out);
    io.write(fmt::format("{}/tet_cube.obj", output_path), surf);
}
