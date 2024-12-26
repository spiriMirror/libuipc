#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("32_different_gravity", "[abd]")
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

    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;  // disable contact

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
        StableNeoHookean       snh;

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        {
            vector<Vector3> Vs = {Vector3{0, 0, 1},
                                  Vector3{0, -1, 0},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](auto& v) { return v * 0.3; });

            auto mesh1 = tetmesh(Vs, Ts);

            label_surface(mesh1);
            label_triangle_orient(mesh1);

            abd.apply_to(mesh1, 100.0_MPa);

            auto trans_view = view(mesh1.transforms());
            auto gravity = mesh1.instances().create<Vector3>(builtin::gravity,
                                                             Vector3::UnitY() * -10.0);

            Transform t     = Transform::Identity();
            t.translation() = Vector3::UnitY() * 1;
            trans_view[0]   = t.matrix();

            object->geometries().create(mesh1);
        }

        {
            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](auto& v) { return v * 0.3; });


            auto mesh2 = tetmesh(Vs, Ts);

            label_surface(mesh2);
            label_triangle_orient(mesh2);

            snh.apply_to(mesh2);
            mesh2.vertices().create<Vector3>(builtin::gravity, Vector3::UnitY() * 10.0);

            object->geometries().create(mesh2);
        }
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}