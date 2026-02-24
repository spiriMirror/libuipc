#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test 1 (Easy): Single tetrahedron falling under gravity with MAS preconditioner.
// This is the simplest possible test to validate MAS init + assemble + apply.
TEST_CASE("53_fem_mas_single_tet", "[fem][mas]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

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

        auto object = scene.objects().create("tet");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 1, 0},
                               Vector3{0, 0, 1},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        auto mesh = tetmesh(Vs, Ts);

        label_surface(mesh);
        label_triangle_orient(mesh);

        // Enable MAS preconditioner via mesh_partition
        mesh_partition(mesh, 16);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.499);
        snh.apply_to(mesh, parm, 1e3);

        object->geometries().create(mesh);
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
