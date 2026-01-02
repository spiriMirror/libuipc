#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>

TEST_CASE("42_abd_fem_subscene", "[abd_fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    SimplicialComplexIO io;

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean       snh;
        AffineBodyConstitution abd;

        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto& subscene_tabular = scene.subscene_tabular();
        auto  default_element  = subscene_tabular.default_element();
        auto  subscene_a       = subscene_tabular.create("subscene_a");
        auto  subscene_b       = subscene_tabular.create("subscene_b");

        // let subscene_a/b be able to interact with the default subscene
        subscene_tabular.insert(default_element, subscene_a, true);
        subscene_tabular.insert(default_element, subscene_b, true);

        auto modelaa = subscene_tabular.at(subscene_a.id(), subscene_a.id());
        REQUIRE(modelaa.is_enabled());
        auto modelbb = subscene_tabular.at(subscene_b.id(), subscene_b.id());
        REQUIRE(modelbb.is_enabled());
        auto modelab = subscene_tabular.at(subscene_a.id(), subscene_b.id());
        REQUIRE(!modelab.is_enabled());


        // create object
        auto object    = scene.objects().create("cubes");
        auto base_mesh = io.read(fmt::format("{}cube.msh", tetmesh_dir));

        label_surface(base_mesh);
        label_triangle_orient(base_mesh);

        SimplicialComplex mesh_a = base_mesh;
        SimplicialComplex mesh_b = base_mesh;

        auto parm = ElasticModuli::youngs_poisson(20.0_kPa, 0.49);
        snh.apply_to(mesh_a, parm);
        subscene_a.apply_to(mesh_a);

        abd.apply_to(mesh_b, 1.0_MPa);
        subscene_b.apply_to(mesh_b);
        auto      trans_view = view(mesh_b.transforms());
        Transform t          = Transform::Identity();
        t.translate(Vector3{0.5, 0, 0.5});  // move a little bit
        trans_view[0] = t.matrix();

        object->geometries().create(mesh_a);
        object->geometries().create(mesh_b);

        auto g = ground(-1.2);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}