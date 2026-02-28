#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test 2 (Medium): Cube mesh falling onto ground with MAS preconditioner.
// Tests: partition of a real mesh, contact, multi-level hierarchy.
TEST_CASE("54_fem_mas_cube_ground", "[fem][mas]")
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

        auto object = scene.objects().create("cube");

        // Scale down
        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0)     = 0.3;
        pre_trans(1, 1)     = 0.3;
        pre_trans(2, 2)     = 0.3;

        SimplicialComplexIO scaled_io(pre_trans);
        auto cube = scaled_io.read(fmt::format("{}/cube.msh", tetmesh_dir));

        label_surface(cube);
        label_triangle_orient(cube);

        // Enable MAS preconditioner
        mesh_partition(cube, 16);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
        snh.apply_to(cube, parm);
        default_element.apply_to(cube);

        object->geometries().create(cube);

        // Ground plane
        auto g = ground(-0.5);
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
