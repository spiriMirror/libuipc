#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test 1 (Easy): Small cube falling under gravity with MAS preconditioner.
// Validates MAS init + assemble + apply on the simplest real mesh (cube.msh).
TEST_CASE("53_fem_mas_small_cube", "[fem][mas]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto        output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                         = test::Scene::default_config();
    config["gravity"]                   = Vector3{0, -9.8, 0};
    config["contact"]["enable"]         = false;
    config["linear_system"]["tol_rate"] = 1e-3;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        StableNeoHookean snh;

        auto object = scene.objects().create("cube");

        Matrix4x4 pre_trans = Matrix4x4::Identity();
        pre_trans(0, 0)     = 0.3;
        pre_trans(1, 1)     = 0.3;
        pre_trans(2, 2)     = 0.3;

        SimplicialComplexIO scaled_io(pre_trans);
        auto mesh = scaled_io.read(fmt::format("{}/cube.msh", tetmesh_dir));

        label_surface(mesh);
        label_triangle_orient(mesh);

        mesh_partition(mesh, 16);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.499);
        snh.apply_to(mesh, parm, 1e3);

        object->geometries().create(mesh);
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
