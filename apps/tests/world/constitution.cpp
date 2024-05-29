#include <catch.hpp>
#include <uipc/world/scene.h>
#include <uipc/common/enumerate.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <iostream>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitutions/affine_body.h>
#include <rapidcsv.h>
#include <uipc/common/format.h>

using namespace uipc;
using namespace uipc::world;
using namespace uipc::constitution;

TEST_CASE("constitution", "[world]")
{
    AffineBodyConstitution        constitution;
    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    auto wood_abd = constitution.create_material(1e8);
    wood_abd.apply_to(mesh0);
}
