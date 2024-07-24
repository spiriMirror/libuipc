#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitutions/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("13_fem3d_gravity", "[fem]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    UIPCEngine engine{"cuda", this_output_path};
    World      world{engine};

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
        auto& snk = scene.constitution_tabular().create<StableNeoHookean>();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 0, 1},
                               Vector3{0, -1, 0},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [&](auto& v) { return v * 0.3; });

        auto mesh = tetmesh(Vs, Ts);

        label_surface(mesh);
        label_triangle_orient(mesh);

        snk.apply_to(mesh);

        object->geometries().create(mesh);
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