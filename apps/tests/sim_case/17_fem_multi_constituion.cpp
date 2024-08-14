#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/arap.h>
#include <filesystem>
#include <fstream>

TEST_CASE("17_fem_multi_constituion", "[fem]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::world;
    using namespace uipc::constitution;
    using namespace uipc::engine;
    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        this_output_path = AssetDir::output_path(__FILE__);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    config["gravity"]                      = Vector3{0, -9.8, 0};
    config["contact"]["enable"]            = true;
    config["line_search"]["max_iter"]      = 8;
    config["linear_system"]["tol_rate"]    = 1e-3;
    config["line_search"]["report_energy"] = false;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean snk;
        scene.constitution_tabular().insert(snk);
        ARAP arap;
        scene.constitution_tabular().insert(arap);

        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto& default_element = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 0, 1},
                               Vector3{0, -1, 0},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        auto mesh = tetmesh(Vs, Ts);

        label_surface(mesh);
        label_triangle_orient(mesh);

        auto mesh2 = mesh;
        auto pos_v = view(mesh2.positions());

        for(auto& p : pos_v)
        {
            p.y() += 1.2;
        }

        auto parm = ElasticModuli::youngs_poisson(10.0_kPa, 0.49);
        snk.apply_to(mesh, parm);

        arap.apply_to(mesh2, 1.0_MPa);

        object->geometries().create(mesh);
        object->geometries().create(mesh2);

        auto g = ground(-1.2);
        object->geometries().create(g);
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 300; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}