#include <app/test_common.h>
#include <app/asset_dir.h>

#include <uipc/world/scene.h>
#include <uipc/geometry.h>
#include <uipc/common/format.h>

using namespace uipc;
using namespace uipc::world;
using namespace uipc::geometry;

TEST_CASE("scene", "[world]")
{
    Scene scene;

    auto object = scene.objects().create("cube");

    SimplicialComplexIO io;
    auto mesh = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    auto [geo, rest_geo] = object->geometries().create(mesh);

    const auto& const_obj = *object;

    auto [const_geo, const_rest_geo] =
        const_obj.geometries().find<SimplicialComplex>(0);

    auto ID = object->id();
    scene.objects().destroy(object->id());

    REQUIRE_ONCE_WARN(scene.objects().destroy(ID));
}
