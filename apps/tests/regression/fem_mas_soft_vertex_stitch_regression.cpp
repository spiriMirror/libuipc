#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/soft_vertex_stitch.h>

// Regression: stitched FEM shells with mesh_partition should stay valid.
// This exercises MAS hierarchy/mapping together with SoftVertexStitch.
TEST_CASE("fem_mas_soft_vertex_stitch_regression", "[fem][mas][stitch][regression]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    auto trimesh_dir = AssetDir::trimesh_path();

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                             = test::Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = false;
    config["contact"]["d_hat"]              = 0.002;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    scene.contact_tabular().default_model(0.01, 1.0_GPa);

    auto default_element = scene.contact_tabular().default_element();
    auto cloth_obj       = scene.objects().create("cloth_pair");

    SimplicialComplexIO io;
    auto cloth_a = io.read(fmt::format("{}cloth80x80.obj", trimesh_dir));
    auto cloth_b = io.read(fmt::format("{}cloth80x80.obj", trimesh_dir));

    {
        auto pos = view(cloth_b.positions());
        for(auto& p : pos)
        {
            p[0] += 0.01;
            p[1] += 0.01;
        }
    }

    label_surface(cloth_a);
    label_surface(cloth_b);
    mesh_partition(cloth_a, 16);
    mesh_partition(cloth_b, 16);

    NeoHookeanShell      nhs;
    DiscreteShellBending dsb;
    auto moduli = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49);
    nhs.apply_to(cloth_a, moduli);
    nhs.apply_to(cloth_b, moduli);
    dsb.apply_to(cloth_a, 10.0);
    dsb.apply_to(cloth_b, 10.0);
    default_element.apply_to(cloth_a);
    default_element.apply_to(cloth_b);

    auto [slot_a, _rest_a] = cloth_obj->geometries().create(cloth_a);
    auto [slot_b, _rest_b] = cloth_obj->geometries().create(cloth_b);

    vector<Vector2i> stitch_pairs;
    auto             n =
        std::min<SizeT>(256, std::min(cloth_a.positions().size(), cloth_b.positions().size()));
    stitch_pairs.reserve(n);
    for(SizeT i = 0; i < n; ++i)
        stitch_pairs.push_back(Vector2i{static_cast<IndexT>(i), static_cast<IndexT>(i)});

    SoftVertexStitch svs;
    auto             stitch_geo = svs.create_geometry({slot_a, slot_b}, stitch_pairs, 1000.0, 0.0);
    auto             stitch_obj = scene.objects().create("stitch");
    stitch_obj->geometries().create(stitch_geo);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    for(int i = 0; i < 3; ++i)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
