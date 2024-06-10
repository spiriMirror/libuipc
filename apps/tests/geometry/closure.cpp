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


    AbstractSimplicialComplex asc;
    asc.vertices().resize(Vs.size());
    asc.tetrahedra().resize(Ts.size());

    auto dst_Ts = geometry::view(asc.tetrahedra());
    std::ranges::copy(Ts, dst_Ts.begin());

    auto mesh = facet_closure(SimplicialComplex{asc, Vs});

    REQUIRE(mesh.triangles().size() == 4);
    REQUIRE(mesh.edges().size() == 6);
}
