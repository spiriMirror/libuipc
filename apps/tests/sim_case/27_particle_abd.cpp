#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/particle.h>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>

TEST_CASE("27_particle_abd", "[abd]")
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

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["friction"]["enable"] = true;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    Scene scene{config};
    {
        // create constitution and contact model
        AffineBodyConstitution abd;
        Particle               pt;

        auto& contact_tabular = scene.contact_tabular();
        contact_tabular.default_model(0.5, 1.0_GPa);
        auto& default_element = contact_tabular.default_element();

        {
            vector<Vector3> Vs = {Vector3{0, 0, 1},
                                  Vector3{0, -1, 0},
                                  Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                  Vector3{std::sqrt(3) / 2, 0, -0.5}};

            vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

            auto      cube_obj = scene.objects().create("board");
            Transform t        = Transform::Identity();
            //t.scale(Vector3{3, 0.1, 3});
            //auto io   = SimplicialComplexIO(t);
            //auto cube = io.read(tetmesh_dir + "cube.msh");

            auto cube = tetmesh(Vs, Ts);
            label_surface(cube);
            label_triangle_orient(cube);
            abd.apply_to(cube, 100.0_MPa);
            auto trans_view = view(cube.transforms());
            t               = Transform(trans_view[0]);
            t.translate(Vector3{0, 1, 0});
            trans_view[0] = t.matrix();
            auto is_fixed = cube.instances().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);
            is_fixed_view[0]   = 1;
            cube_obj->geometries().create(cube);
        }


        auto            particle_obj = scene.objects().create("particle");
        vector<Vector3> Vs           = {Vector3{0, 1.5, 0}};
        auto            particle     = pointcloud(Vs);
        label_surface(particle);
        pt.apply_to(particle, 1e3, 0.2);
        default_element.apply_to(particle);
        particle_obj->geometries().create(particle);
    }

    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 200; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}