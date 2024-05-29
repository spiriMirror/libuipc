#include <catch.hpp>
#include <uipc/world/scene.h>
#include <uipc/common/enumerate.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <iostream>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/format.h>

using namespace uipc;
using namespace uipc::world;

TEST_CASE("contact_model", "[world]")
{
    Scene scene;
    auto& contact_tabular = scene.contact_tabular();

    auto default_element = contact_tabular.default_element();
    auto wood_contact    = contact_tabular.create();
    auto rubber_contact  = contact_tabular.create();

    contact_tabular.default_model(0.5, 1e8);
    contact_tabular.insert(wood_contact, wood_contact, 0.5, 1e8);
    contact_tabular.insert(rubber_contact, rubber_contact, 0.6, 1e8);
    contact_tabular.insert(wood_contact, rubber_contact, 0.3, 1e8);

    vector<Vector2i> gt = {{0, 0}, {1, 1}, {1, 2}, {2, 2}};

    auto contact_models = contact_tabular.contact_models();

    REQUIRE(std::ranges::equal(gt,
                               contact_models,
                               [](const Vector2i& a, const ContactModel& b)
                               { return a == b.ids(); }));

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));

    wood_contact.apply_to(mesh0);

    auto contact_element = mesh0.meta().find<IndexT>(builtin::contact_element_id);
    REQUIRE(contact_element);
    REQUIRE(contact_element->view().front() == wood_contact.id());

    auto mesh1 = mesh0;
    rubber_contact.apply_to(mesh1);
    REQUIRE(mesh1.meta().find<IndexT>(builtin::contact_element_id)->view().front()
            == rubber_contact.id());

    auto mesh2 = mesh0;
    default_element.apply_to(mesh2);
    REQUIRE(default_element.name() == "default");
    REQUIRE(mesh2.meta().find<IndexT>(builtin::contact_element_id)->view().front() == 0);

    //Json j = contact_tabular;
    //std::cout << j.dump(4) << std::endl;
}
