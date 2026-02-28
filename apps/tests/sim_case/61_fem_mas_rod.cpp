#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/constitution/kirchhoff_rod_bending.h>

// Test: Rod (1D line mesh) with MAS preconditioner.
// Multiple rods fixed at one end, bending under gravity.
TEST_CASE("61_fem_mas_rod", "[fem][mas]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["linear_system"]["tol_rate"]     = 1e-3;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        HookeanSpring       hs;
        KirchhoffRodBending krb;
        auto default_contact = scene.contact_tabular().default_element();

        auto object = scene.objects().create("rods");

        constexpr int   n = 20;  // nodes per rod
        constexpr int   num_rods = 3;

        for(int r = 0; r < num_rods; r++)
        {
            vector<Vector3> Vs(n);
            for(int i = 0; i < n; i++)
                Vs[i] = Vector3{r * 0.1, 0.3, i * 0.03};

            vector<Vector2i> Es(n - 1);
            for(int i = 0; i < n - 1; i++)
                Es[i] = Vector2i{i, i + 1};

            auto mesh = linemesh(Vs, Es);
            label_surface(mesh);

            // Partition for MAS
            mesh_partition(mesh, 16);

            hs.apply_to(mesh, 10.0_MPa);
            krb.apply_to(mesh, 1.0_MPa);
            default_contact.apply_to(mesh);

            // Fix first two nodes
            auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
            auto is_fixed_view = view(*is_fixed);
            is_fixed_view[0] = 1;
            is_fixed_view[1] = 1;

            object->geometries().create(mesh);
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    for(int i = 0; i < 20; i++)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
