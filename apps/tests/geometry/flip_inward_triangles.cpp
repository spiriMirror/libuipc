#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/geometry/utils/closure.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("flip_inward_triangles", "[surface]")
{
    fmt::println("----------------------------------------------------------------------------");
    fmt::println(">>> flip_inward_triangles >>>");

    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto mesh = tetmesh(Vs, Ts);
    fmt::println("mesh:\n {}", mesh);
    auto labeled = label_surface(mesh);
    fmt::println("labeled:\n {}", labeled);
    auto oriented = label_triangle_orient(labeled);
    fmt::println("oriented:\n {}", oriented);
    auto flipped = flip_inward_triangles(oriented);
    fmt::println("flipped:\n {}", flipped);
    auto surface = extract_surface(flipped);
    fmt::println("surface:\n {}", surface);

    fmt::println("----------------------------------------------------------------------------");
}
