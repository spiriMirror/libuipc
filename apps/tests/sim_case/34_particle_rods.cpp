#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/constitution/particle.h>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>

TEST_CASE("34_particle_rods", "[abd]")
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

    auto config                 = Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        HookeanSpring hs;
        Particle      pt;

        auto& contact_tabular = scene.contact_tabular();
        contact_tabular.default_model(0.5, 1.0_GPa);
        auto& default_element = contact_tabular.default_element();

        auto particle_obj = scene.objects().create("particle");
        {
            vector<Vector3> Vs       = {Vector3{0, 1.5, 0}};
            auto            particle = pointcloud(Vs);
            label_surface(particle);
            pt.apply_to(particle, 1e3, 0.2);
            default_element.apply_to(particle);
            particle_obj->geometries().create(particle);
        }


        auto rods_obj = scene.objects().create("rods");
        {
            vector<Vector3> Vs = {
                Vector3{0, 0, 1}, Vector3{0, 0, 2}, Vector3{0, 0, 3}, Vector3{0, 0, 4}};

            vector<Vector2i> Es = {{0, 1}, {1, 2}, {2, 3}};

            auto rods = linemesh(Vs, Es);
            label_surface(rods);
            hs.apply_to(rods, 1e3, 0.2);
            default_element.apply_to(rods);
            rods_obj->geometries().create(rods);
        }
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 10)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}