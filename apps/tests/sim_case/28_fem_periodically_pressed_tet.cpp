#include <catch.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <filesystem>
#include <fstream>
#include <numbers>

TEST_CASE("28_fem_periodically_pressed_tet", "[animation]")
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

    config["gravity"] = Vector3{0, -9.8, 0};

    {  // dump config
        std::ofstream ofs(fmt::format("{}config.json", this_output_path));
        ofs << config.dump(4);
    }

    SimplicialComplexIO io;

    Scene scene{config};

    // create constitution and contact model
    StableNeoHookean       snh;
    SoftPositionConstraint spc;
    scene.constitution_tabular().insert(snh);
    scene.constitution_tabular().insert(spc);

    // create object
    auto object = scene.objects().create("tets");

    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
    vector<Vector3>  Vs = {Vector3{0, 1, 0},
                           Vector3{0, 0, 1},
                           Vector3{-std::sqrt(3) / 2, 0, -0.5},
                           Vector3{std::sqrt(3) / 2, 0, -0.5}};

    auto tet = tetmesh(Vs, Ts);

    geometry::label_surface(tet);
    geometry::label_triangle_orient(tet);

    auto parm = ElasticModuli::youngs_poisson(1e5, 0.499);
    snh.apply_to(tet, parm, 1e3);
    spc.apply_to(tet, 100.0);
    object->geometries().create(tet);

    auto ground_object = scene.objects().create("ground");
    auto g             = ground(-0.5);
    ground_object->geometries().create(g);

    auto& animator = scene.animator();
    animator.insert(
        *object,
        [](Animation::UpdateInfo& info)
        {
            auto geo_slots = info.geo_slots();
            auto geo       = geo_slots[0]->geometry().as<SimplicialComplex>();
            auto rest_geo_slots = info.rest_geo_slots();
            auto rest_geo = rest_geo_slots[0]->geometry().as<SimplicialComplex>();

            auto is_constrained = geo->vertices().find<IndexT>(builtin::is_constrained);
            auto is_constrained_view = view(*is_constrained);
            auto aim_position = geo->vertices().find<Vector3>(builtin::aim_position);
            auto aim_position_view  = view(*aim_position);
            auto rest_position_view = rest_geo->positions().view();

            is_constrained_view[0] = 1;

            auto t     = info.dt() * info.frame();
            auto theta = std::numbers::pi * t;
            auto y     = -std::sin(theta);

            aim_position_view[0] = rest_position_view[0] + Vector3::UnitY() * y;
        });


    world.init(scene); REQUIRE(world.is_valid());
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    while(world.frame() < 360)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
    }
}