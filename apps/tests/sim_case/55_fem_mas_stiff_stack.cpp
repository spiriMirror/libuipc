#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test 3 (Hard): Stack of stiff cubes with MAS preconditioner.
// Tests: large DOF count, multi-body contact, stiff material (high Young's modulus).
// This is where MAS should show significant iteration reduction over diagonal.
TEST_CASE("55_fem_mas_stiff_stack", "[fem][mas]")
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

    SimplicialComplexIO io;

    Scene scene{config};
    {
        StableNeoHookean snh;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        auto object = scene.objects().create("stack");

        // Scale down
        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0)     = 0.2;
        pre_trans(1, 1)     = 0.2;
        pre_trans(2, 2)     = 0.2;

        SimplicialComplexIO scaled_io(pre_trans);
        auto cube = scaled_io.read(fmt::format("{}/cube.msh", tetmesh_dir));

        label_surface(cube);
        label_triangle_orient(cube);

        // Enable MAS preconditioner
        mesh_partition(cube, 16);

        // Stiff material (1 MPa) - this is where MAS should shine vs diagonal
        auto parm = ElasticModuli::youngs_poisson(1.0_MPa, 0.49);
        snh.apply_to(cube, parm);
        default_element.apply_to(cube);

        // Stack 5 cubes
        constexpr int N = 5;
        for(int i = 0; i < N; i++)
        {
            auto pos = view(cube.positions());
            for(auto& p : pos)
                p[1] += 0.24;
            object->geometries().create(cube);
        }

        // Ground plane
        auto g = ground(0.0);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
