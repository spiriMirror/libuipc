#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>


using namespace uipc;
using namespace uipc::core;
using namespace uipc::constitution;

TEST_CASE("object", "[object]")
{
    Scene                  scene;
    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

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
    Scene                  scene;
    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

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