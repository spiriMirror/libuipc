#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/empty.h>

// Regression: MAS going_next buffer overlap when unpartitioned (Empty) vertices
// push cloth real indices beyond the padded partition space.
//
// Setup: 3x3 quad grid cloth (16 vertices, 1 partition of 16, zero waste)
//      + 1 Empty tet (4 vertices, unpartitioned)
//      = 20 total FEM vertices.
//
// Without fix: level1_begin = padded(16) = 16, but cloth real indices reach 19,
//              so going_next[16..19] collides between level-0 and level-1.
// With fix:    level1_begin = max(16, 20) = 20, no overlap.
TEST_CASE("81_fem_mas_cloth_with_empty", "[fem][mas][regression]")
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
        // ---- Cloth: 3x3 quad grid = 16 vertices, 18 triangles ----
        NeoHookeanShell nhs;

        auto cloth_obj = scene.objects().create("cloth");

        constexpr int   N          = 3;
        constexpr Float cloth_size = 0.3;
        constexpr Float spacing    = cloth_size / N;

        vector<Vector3>  Vs;
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

        auto cloth_mesh = trimesh(Vs, Fs);
        label_surface(cloth_mesh);
        mesh_partition(cloth_mesh, 16);

        auto parm = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49);
        nhs.apply_to(cloth_mesh, parm);

        auto is_fixed = cloth_mesh.vertices().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);
        is_fixed_view[0]   = 1;
        is_fixed_view[N]   = 1;

        cloth_obj->geometries().create(cloth_mesh);

        // ---- Empty: 1 tetrahedron = 4 vertices (unpartitioned) ----
        Empty empty;

        vector<Vector4i> Ts_empty = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs_empty = {Vector3{0.5, 0.0, 0.0},
                                     Vector3{0.5, 0.0, 0.1},
                                     Vector3{0.6, 0.0, 0.0},
                                     Vector3{0.55, 0.1, 0.05}};

        auto empty_mesh = tetmesh(Vs_empty, Ts_empty);
        label_surface(empty_mesh);
        label_triangle_orient(empty_mesh);
        empty.apply_to(empty_mesh);

        cloth_obj->geometries().create(empty_mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    for(int i = 0; i < 5; i++)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
