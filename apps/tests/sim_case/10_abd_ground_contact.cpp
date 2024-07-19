#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitutions/affine_body.h>
#include <filesystem>
#include <fstream>

TEST_CASE("10_abd_ground_contact", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    UIPCEngine engine{"cuda", this_output_path};
    World      world{engine};

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

        // create object
        auto object = scene.objects().create("tet");
        {
            vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
            vector<Vector3>  Vs = {Vector3{0, 1, 0},
                                   Vector3{0, 0, 1},
                                   Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                   Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](auto& v) { return v * 0.3; });

            auto tet = tetmesh(Vs, Ts);

            label_surface(tet);
            label_triangle_orient(tet);
            abd.apply_to(tet, 100.0_MPa);

            object->geometries().create(tet);
        }

        // create a ground geometry
        ImplicitGeometry half_plane = ground(-1.0);

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