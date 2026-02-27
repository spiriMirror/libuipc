#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

// Test: Two separately partitioned bunny meshes in the same scene.
// Exposes the partition ID offset bug: both meshes have local partition IDs
// starting at 0. Without a global offset, vertices from different meshes
// get merged into the same cluster, violating the BANKSIZE invariant.
// The bunny has ~1869 vertices, so METIS creates ~117 partitions per mesh,
// and partition IDs 0..116 collide between the two copies.
TEST_CASE("57_fem_mas_two_bunnies", "[fem][mas]")
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

        auto object = scene.objects().create("two_bunnies");

        SimplicialComplexIO io;

        // Bunny A — partitioned
        {
            auto bunny_a = io.read(fmt::format("{}/bunny0.msh", tetmesh_dir));
            label_surface(bunny_a);
            label_triangle_orient(bunny_a);
            mesh_partition(bunny_a, 16);

            auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
            snh.apply_to(bunny_a, parm);
            default_element.apply_to(bunny_a);

            object->geometries().create(bunny_a);
        }

        // Bunny B — also partitioned, shifted up
        {
            auto bunny_b = io.read(fmt::format("{}/bunny0.msh", tetmesh_dir));
            label_surface(bunny_b);
            label_triangle_orient(bunny_b);
            mesh_partition(bunny_b, 16);

            auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
            snh.apply_to(bunny_b, parm);
            default_element.apply_to(bunny_b);

            auto pos = view(bunny_b.positions());
            for(auto& p : pos) p[1] += 2.0;

            object->geometries().create(bunny_b);
        }

        // Ground
        auto g = ground(-1.0);
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
