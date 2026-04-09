#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/finite_element_external_force.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>

TEST_CASE("45_finite_element_external_vertex_force", "[fem][external_force]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    namespace fs = std::filesystem;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, 0, 0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        StableNeoHookean          snh;
        FiniteElementExternalForce ext_force;

        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0)     = 0.2;
        pre_trans(1, 1)     = 0.2;
        pre_trans(2, 2)     = 0.2;

        SimplicialComplexIO io{pre_trans};
        auto cube = io.read(fmt::format("{}/cube.msh", tetmesh_dir));

        label_surface(cube);
        label_triangle_orient(cube);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.499);
        snh.apply_to(cube, parm, 1e3);

        ext_force.apply_to(cube, Vector3::Zero());

        auto object = scene.objects().create("cube");
        object->geometries().create(cube);

        scene.animator().insert(
            *object,
            [](Animation::UpdateInfo& info)
            {
                Float time = info.dt() * info.frame();

                Float force_magnitude = 50.0;
                Float orbit_speed     = 0.5;

                Float   orbit_angle = orbit_speed * time;
                Vector3 force{std::cos(orbit_angle) * force_magnitude,
                              0.0,
                              std::sin(orbit_angle) * force_magnitude};

                for(auto& geo_slot : info.geo_slots())
                {
                    auto& geo = geo_slot->geometry();
                    auto* sc  = geo.as<SimplicialComplex>();
                    if(!sc)
                        continue;

                    auto force_attr = sc->vertices().find<Vector3>("external_force");
                    if(!force_attr)
                        continue;

                    auto is_constrained =
                        sc->vertices().find<IndexT>(builtin::is_constrained);
                    if(is_constrained)
                    {
                        auto is_constrained_view = view(*is_constrained);
                        std::ranges::fill(is_constrained_view, 1);
                    }

                    auto force_view = view(*force_attr);
                    std::ranges::fill(force_view, force);
                }
            });
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 100)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
