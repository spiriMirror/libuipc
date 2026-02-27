#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>

// Test: Cloth (2D triangle mesh) with MAS preconditioner.
// A sheet of cloth fixed at two corners, sagging under gravity.
TEST_CASE("60_fem_mas_cloth", "[fem][mas]")
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
        NeoHookeanShell nhs;
        auto default_contact = scene.contact_tabular().default_element();

        auto object = scene.objects().create("cloth");

        // Build a grid cloth mesh (N x N quads -> 2*N*N triangles)
        constexpr int N = 10;
        constexpr Float cloth_size = 0.5;
        constexpr Float spacing = cloth_size / N;

        vector<Vector3> Vs;
        vector<Vector3i> Fs;

        for(int i = 0; i <= N; i++)
            for(int j = 0; j <= N; j++)
                Vs.push_back(Vector3{i * spacing, 0.5, j * spacing});

        for(int i = 0; i < N; i++)
        {
            for(int j = 0; j < N; j++)
            {
                int v00 = i * (N + 1) + j;
                int v10 = (i + 1) * (N + 1) + j;
                int v01 = i * (N + 1) + (j + 1);
                int v11 = (i + 1) * (N + 1) + (j + 1);
                Fs.push_back(Vector3i{v00, v10, v11});
                Fs.push_back(Vector3i{v00, v11, v01});
            }
        }

        auto mesh = trimesh(Vs, Fs);
        label_surface(mesh);

        // Partition for MAS
        mesh_partition(mesh, 16);

        auto parm = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49);
        nhs.apply_to(mesh, parm);
        default_contact.apply_to(mesh);

        // Fix two corners
        auto is_fixed      = mesh.vertices().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);
        is_fixed_view[0]         = 1;  // corner (0,0)
        is_fixed_view[N]         = 1;  // corner (0,N)

        object->geometries().create(mesh);
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
