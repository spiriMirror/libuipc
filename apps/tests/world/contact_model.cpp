#include <catch.hpp>
#include <uipc/world/scene.h>
#include <uipc/common/enumerate.h>
#include <app/asset_dir.h>
#include <uipc/geometry.h>
#include <uipc/world/apply_contact_element.h>
#include <iostream>

using namespace uipc;
using namespace uipc::world;

TEST_CASE("contact_model", "[world]")
{
    Scene scene;
    auto& ct             = scene.contact_tabular();
    auto  wood_contact   = ct.create();
    auto  rubber_contact = ct.create();

    ct.default_model(0.5, 1e8);
    ct.insert(wood_contact, wood_contact, 0.5, 1e8);
    ct.insert(rubber_contact, rubber_contact, 0.5, 1e8);
    ct.insert(wood_contact, rubber_contact, 0.5, 1e8);

    vector<Vector2i> gt = {{0, 0}, {1, 1}, {1, 2}, {2, 2}};

    auto contact_models = ct.contact_models();

    REQUIRE(std::equal(gt.begin(),
                       gt.end(),
                       contact_models.begin(),
                       [](const Vector2i& a, const ContactModel& b)
                       { return a == b.ids(); }));

    geometry::SimplicialComplexIO io;
    auto mesh = io.read_msh(std::format("{}cube.msh", AssetDir::tetmesh_path()));

    apply(wood_contact, mesh);

    auto contact_element = mesh.meta().find<IndexT>("contact_element_id");
    REQUIRE(contact_element);
    REQUIRE(contact_element->view().front() == wood_contact.id());

    //Json j = ct;
    //std::cout << j.dump(4) << std::endl;
}
