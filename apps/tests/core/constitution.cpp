#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("abd", "[constitution]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::constitution;

    Scene                  scene;
    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    mesh0.instances().resize(5);

    auto wood_abd = abd.create_material(1e8);
    wood_abd.apply_to(mesh0);

    auto kappa = mesh0.instances().find<Float>("kappa");

    REQUIRE(kappa->size() == mesh0.instances().size());
    REQUIRE(std::ranges::all_of(kappa->view(), [](auto v) { return v == 1e8; }));

    // this name is important for the AffineBodyConstitution
    REQUIRE(kappa->name() == "kappa");
}
