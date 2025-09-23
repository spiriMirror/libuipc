#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("42_abd_fem_subscene", "[abd_fem]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean       snh;
        AffineBodyConstitution abd;

        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();
        auto subscene_a      = scene.subscene_tabular().create("subscene_a");
        auto subscene_b      = scene.subscene_tabular().create("subscene_b");
        scene.subscene_tabular().insert(subscene_a, subscene_b, false);

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

        object->geometries().create(mesh_a);
        object->geometries().create(mesh_b);

        auto g = ground(-1.2);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 100)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}