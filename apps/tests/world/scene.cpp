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

    auto ID = object->id();
    scene.objects().destroy(ID);

    REQUIRE_ONCE_WARN(scene.objects().destroy(ID));

    REQUIRE(geo.expired());
    REQUIRE(rest_geo.expired());

    object = scene.objects().create("cube1");

    // create two geometries
    object->geometries().create(mesh);
    object->geometries().create(mesh);

    REQUIRE(object->geometries().ids().size() == 2);
    REQUIRE(object->geometries().ids()[0] == 1);
    REQUIRE(object->geometries().ids()[1] == 2);
}
