#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <uipc/geometry/utils/closure.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("closure", "[closure]")
{
    std::vector           Vs = {Vector3{0.0, 0.0, 0.0},
                                Vector3{1.0, 0.0, 0.0},
                                Vector3{0.0, 1.0, 0.0},
                                Vector3{0.0, 0.0, 1.0}};
    std::vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto mesh = tetmesh(Vs, Ts);

    auto c = closure(mesh);

    REQUIRE(c.positions().size() == mesh.positions().size());
    REQUIRE(c.tetrahedra().size() == mesh.tetrahedra().size());

    REQUIRE(c.triangles().size() == 4);
    REQUIRE(c.edges().size() == 6);
}
