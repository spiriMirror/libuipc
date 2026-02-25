#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test 4: Bunny mesh falling onto ground with MAS preconditioner.
// Tests: larger mesh (~1000+ verts), multi-cluster partitioning, contact.
TEST_CASE("56_fem_mas_bunny_ground", "[fem][mas]")
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

        auto object = scene.objects().create("bunny");

        SimplicialComplexIO io;
        auto bunny = io.read(fmt::format("{}/bunny0.msh", tetmesh_dir));

        label_surface(bunny);
        label_triangle_orient(bunny);

        // Enable MAS preconditioner
        mesh_partition(bunny, 16);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
        snh.apply_to(bunny, parm);
        default_element.apply_to(bunny);

        object->geometries().create(bunny);

        // Ground plane below bunny
        auto g = ground(-1.0);
        object->geometries().create(g);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    for(int i = 0; i < 50; i++)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
