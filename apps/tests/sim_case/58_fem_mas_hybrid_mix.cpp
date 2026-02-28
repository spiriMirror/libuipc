#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test: One mesh WITH mesh_partition() + one mesh WITHOUT.
// Tests the hybrid MAS+Diag path.
// Before fix: current "all or nothing" logic rejects the scene
//             (FEMDiagPreconditioner defers, FEMMASPreconditioner requires all).
// After fix: MAS handles partitioned mesh, diagonal handles unpartitioned mesh.
TEST_CASE("58_fem_mas_hybrid_mix", "[fem][mas]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        StableNeoHookean snh;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        auto object = scene.objects().create("hybrid");

        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0) = 0.2;
        pre_trans(1, 1) = 0.2;
        pre_trans(2, 2) = 0.2;

        SimplicialComplexIO scaled_io(pre_trans);

        // Mesh A — WITH partition (MAS)
        {
            auto cube_a = scaled_io.read(fmt::format("{}/cube.msh", tetmesh_dir));
            label_surface(cube_a);
            label_triangle_orient(cube_a);
            mesh_partition(cube_a, 16);  // <-- partitioned

            auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
            snh.apply_to(cube_a, parm);
            default_element.apply_to(cube_a);

            auto pos = view(cube_a.positions());
            for(auto& p : pos) p[1] += 0.5;

            object->geometries().create(cube_a);
        }

        // Mesh B — WITHOUT partition (should use diagonal fallback)
        {
            auto cube_b = scaled_io.read(fmt::format("{}/cube.msh", tetmesh_dir));
            label_surface(cube_b);
            label_triangle_orient(cube_b);
            // NOTE: no mesh_partition() call

            auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
            snh.apply_to(cube_b, parm);
            default_element.apply_to(cube_b);

            auto pos = view(cube_b.positions());
            for(auto& p : pos) p[1] += 1.0;

            object->geometries().create(cube_b);
        }

        // Ground
        auto g = ground(0.0);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    for(int i = 0; i < 10; i++)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
