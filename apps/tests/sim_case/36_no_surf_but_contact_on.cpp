#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>

TEST_CASE("36_no_surf_but_contact_on", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    using namespace uipc::core;
    namespace fs = std::filesystem;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;
    test::Scene::dump_config(config, output_path);

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
        sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0)));


    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        REQUIRE_HAS_WARN(sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame())));
    }
}
