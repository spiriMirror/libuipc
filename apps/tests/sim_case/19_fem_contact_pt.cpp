#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("19_fem_contact_pt", "[fem]")
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

    auto config                = Scene::default_config();
    config["gravity"]          = Vector3{0, -9.8, 0};
    config["contact"]["d_hat"] = 0.01;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        StableNeoHookean snk;
        scene.constitution_tabular().insert(snk);
        auto& default_contact = scene.contact_tabular().default_element();

        // create object
        auto object = scene.objects().create("tets");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

        {
            vector<Vector3> Vs = {Vector3{0, 1, 0},
                                  Vector3{0, 0, 1},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            std::transform(Vs.begin(),
                           Vs.end(),
                           Vs.begin(),
                           [&](const Vector3& v)
                           { return v * 0.3 + Vector3::UnitY() * 0.35; });

            auto mesh1 = tetmesh(Vs, Ts);

            label_surface(mesh1);
            label_triangle_orient(mesh1);

            auto parm = ElasticModuli::youngs_poisson(10.0_kPa, 0.49);
            snk.apply_to(mesh1, parm);
            default_contact.apply_to(mesh1);

            auto is_fixed = mesh1.vertices().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);
            is_fixed_view[0]   = 0;


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

            auto parm = ElasticModuli::youngs_poisson(10.0_kPa, 0.49);
            snk.apply_to(mesh2, parm);

            auto is_fixed = mesh2.vertices().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);
            std::ranges::fill(is_fixed_view, 1);

            object->geometries().create(mesh2);
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