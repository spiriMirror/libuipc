#include <catch.hpp>
#include <app/asset_dir.h>
#include <app/require_log.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("36_no_surf_but_contact_on", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    auto this_output_path = AssetDir::output_path(__FILE__);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                 = Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        scene.contact_tabular().default_model(0.5, 1.0_GPa);

        Transform pre_transform = Transform::Identity();
        pre_transform.scale(0.3);
        SimplicialComplexIO io{pre_transform};

        // create object
        auto abd_object = scene.objects().create("abd");
        {
            AffineBodyConstitution abd;
            SimplicialComplex      abd_mesh =
                io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
            abd.apply_to(abd_mesh, 100.0_MPa);
            abd_object->geometries().create(abd_mesh);
        }

        auto snh_object = scene.objects().create("fem");
        {
            StableNeoHookean  snh;
            SimplicialComplex snh_mesh =
                io.read(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
            snh.apply_to(snh_mesh);
            snh_object->geometries().create(snh_mesh);
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());
    SceneIO sio{scene};

    REQUIRE_HAS_WARN(
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0)));


    while(world.frame() < 50)
    {
        world.advance();
        world.retrieve();
        REQUIRE_HAS_WARN(sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame())));
    }
}