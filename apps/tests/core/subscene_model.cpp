#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

TEST_CASE("subscene_model", "[subscene_model]")
{
    using namespace uipc;
    using namespace uipc::core;

    Scene scene;
    auto& subscene_tabular = scene.subscene_tabular();

    auto default_element = subscene_tabular.default_element();
    auto scene_a         = subscene_tabular.create();
    auto scene_b         = subscene_tabular.create();
    subscene_tabular.insert(scene_a, scene_b, false);

    auto subscene_models = subscene_tabular.subscene_models();

    REQUIRE(subscene_models.find<Vector2i>("topo"));
    REQUIRE(subscene_models.find<IndexT>("is_enabled"));

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    scene_a.apply_to(mesh0);

    auto contact_element = mesh0.meta().find<IndexT>(builtin::subscene_element_id);
    REQUIRE(contact_element);
    REQUIRE(contact_element->view().front() == scene_a.id());

    auto mesh1 = mesh0;
    scene_b.apply_to(mesh1);
    REQUIRE(mesh1.meta().find<IndexT>(builtin::subscene_element_id)->view().front()
            == scene_b.id());

    auto mesh2 = mesh0;
    default_element.apply_to(mesh2);
    REQUIRE(default_element.name() == "default");
    REQUIRE(mesh2.meta().find<IndexT>(builtin::subscene_element_id)->view().front() == 0);
}

