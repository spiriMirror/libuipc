#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <filesystem>
#include <fstream>

TEST_CASE("22_fem_animated_vertices", "[animation]")
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

    Float dt                            = 0.01;
    config["gravity"]                   = Vector3{0, -9.8, 0};
    config["contact"]["enable"]         = false;  // disable contact
    config["line_search"]["max_iter"]   = 8;
    config["linear_system"]["tol_rate"] = 1e-3;
    config["dt"]                        = dt;

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};

    // create constitution and contact model
    StableNeoHookean snh;
    scene.constitution_tabular().insert(snh);

    // create object
    auto object = scene.objects().create("tets");

    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
    vector<Vector3>  Vs = {Vector3{0, 1, 0},
                           Vector3{0, 0, 1},
                           Vector3{-std::sqrt(3) / 2, 0, -0.5},
                           Vector3{std::sqrt(3) / 2, 0, -0.5}};

    auto mesh = tetmesh(Vs, Ts);

    geometry::label_surface(mesh);
    geometry::label_triangle_orient(mesh);

    auto parm = ElasticModuli::youngs_poisson(1e5, 0.499);
    snh.apply_to(mesh, parm, 1e3);

    auto arr = mesh.tetrahedra().find<Float>("mu");

    object->geometries().create(mesh);

    auto& animator = world.animator();
    animator.insert(*object,
                    [=](Animation::UpdateInfo& info)
                    {
                        auto geo_slots = info.geo_slots();
                        auto geo = geo_slots[0]->geometry().as<SimplicialComplex>();

                        auto aim = geo->vertices().find<Vector3>(builtin::aim_position);
                        auto aim_view = view(*aim);

                        auto sin_t = std::sin(dt * info.frame());
                        auto cos_t = std::cos(dt * info.frame());

                        // move the first vertex in a circle
                        aim_view[0] = Vector3{0, cos_t, sin_t};
                    });


    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    for(int i = 1; i < 360; i++)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, i));
    }
}