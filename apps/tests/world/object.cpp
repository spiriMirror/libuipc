#include <app/test_common.h>

#include <app/asset_dir.h>
#include <iostream>
#include <uipc/common/enumerate.h>
#include <uipc/geometry.h>
#include <uipc/world/scene.h>

#include <uipc/builtin/attribute_name.h>
#include <uipc/common/format.h>
#include <uipc/constitutions/affine_body.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/world/object.h>


using namespace uipc;
using namespace uipc::world;
using namespace uipc::constitution;

TEST_CASE("object", "[object]")
{
    Scene scene;
    auto& constitution_tabular = scene.constitution_tabular();
    auto& abd = constitution_tabular.create<AffineBodyConstitution>();

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto wood_abd = abd.create_material(1e8);

    auto obj             = scene.objects().create("cube");
    auto [geo, rest_geo] = obj->geometries().create(mesh0);


    auto V      = geo->geometry().positions().view();
    auto V_rest = rest_geo->geometry().positions().view();

    // if we add a mesh as geometries and the rest geometries in this way
    // anything will be shared
    REQUIRE(geo->geometry().positions().is_shared());
    REQUIRE(rest_geo->geometry().positions().is_shared());
    REQUIRE(std::ranges::equal(V, V_rest));
}

TEST_CASE("const_object", "[object]")
{
    Scene scene;
    auto& constitution_tabular = scene.constitution_tabular();
    auto& abd = constitution_tabular.create<AffineBodyConstitution>();

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto wood_abd = abd.create_material(1e8);

    auto obj = scene.objects().create("cube");

    auto [geo, rest_geo] = obj->geometries().create(mesh0);

    auto V      = geo->geometry().positions().view();
    auto V_rest = rest_geo->geometry().positions().view();

    UNUSED geo->geometry().positions();
    UNUSED geo->geometry().vertices();
    UNUSED geo->geometry().edges();
    UNUSED geo->geometry().triangles();
    UNUSED geo->geometry().tetrahedra();
}