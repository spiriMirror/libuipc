#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("flip_inward_triangles", "[surface]")
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto mesh = tetmesh(Vs, Ts);
    label_surface(mesh);
    auto orient = label_triangle_orient(mesh);

    // there are some orient are inverted (< 0)
    REQUIRE(std::ranges::count(orient->view(), -1) > 0);
    auto flipped = flip_inward_triangles(mesh);
    // after flipping, all the orient should be positive
    auto flipped_orient = flipped.triangles().find<IndexT>(builtin::orient);
    REQUIRE(std::ranges::all_of(flipped_orient->view(),
                                [](auto o) { return o > 0; }));
}
