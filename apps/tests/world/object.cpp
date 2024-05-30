#include <catch.hpp>
#include <uipc/world/scene.h>
#include <uipc/common/enumerate.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <iostream>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitutions/affine_body.h>
#include <uipc/common/format.h>
#include <uipc/world/object.h>

using namespace uipc;
using namespace uipc::world;
using namespace uipc::constitution;

TEST_CASE("obj", "[world]")
{
    Scene scene;
    auto& constitution_tabular = scene.constitution_tabular();
    auto& abd = constitution_tabular.create<AffineBodyConstitution>();

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto wood_abd = abd.create_material(1e8);

    Object obj;
    obj.push_back(mesh0);

    auto  sc       = obj.find<geometry::SimplicialComplex>();
    auto& geo      = sc.geometries().view()[0];
    auto& rest_geo = sc.rest_geometries().view()[0];

    auto V      = geo.positions().view();
    auto V_rest = rest_geo.positions().view();

    // if we add a mesh as geometries and the rest geometries in this way
    // anything will be shared
    REQUIRE(geo.positions().is_shared());
    REQUIRE(rest_geo.positions().is_shared());

    // but the geometries should not be the same pointer
    REQUIRE(std::ranges::equal(V, V_rest));
}

TEST_CASE("const_obj", "[world]")
{
    Scene scene;
    auto& constitution_tabular = scene.constitution_tabular();
    auto& abd = constitution_tabular.create<AffineBodyConstitution>();

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto wood_abd = abd.create_material(1e8);

    Object obj;

    auto sc = obj.push_back(mesh0);

    auto& geo      = sc.geometries().view()[0];
    auto& rest_geo = sc.rest_geometries().view()[0];

    auto V      = geo.positions().view();
    auto V_rest = rest_geo.positions().view();

    // if we add a mesh as geometries and the rest geometries in this way
    // anything will be shared
    REQUIRE(geo.positions().is_shared());
    REQUIRE(rest_geo.positions().is_shared());

    // but the geometries should not be the same pointer
    REQUIRE(std::ranges::equal(V, V_rest));

    const auto& const_obj = obj;

    auto const_sc = const_obj.find<geometry::SimplicialComplex>();
    const_sc.geometries().view();
}