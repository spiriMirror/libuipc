#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitutions/affine_body.h>
#include <filesystem>
#include <fstream>

TEST_CASE("12_abd_self_ground_contact", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config       = Scene::default_config();
    config["gravity"] = Vector3{0, -9.8, 0};

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        auto& abd = scene.constitution_tabular().create<AffineBodyConstitution>();
        auto default_contact = scene.contact_tabular().default_element();

        Transform pre_trans = Transform::Identity();
        pre_trans.scale(0.3);
        SimplicialComplexIO io{pre_trans};
        auto cube = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

        // create object
        auto object = scene.objects().create("cubes");
        {

            label_surface(cube);
            label_triangle_orient(cube);

            cube.instances().resize(2);

            auto trans = view(cube.transforms());

            Transform t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.2);
            trans[0] = t.matrix();

            t = Transform::Identity();
            t.translate(Vector3::UnitY() * 0.6);
            trans[1] = t.matrix();

            abd.apply_to(cube, 100.0_MPa);

            object->geometries().create(cube);
        }

        // create a ground geometry
        ImplicitGeometry half_plane = ground(0.0);

        auto ground = scene.objects().create("ground");
        {
            ground->geometries().create(half_plane);
        }
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 50; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}