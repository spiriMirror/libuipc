#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/empty.h>
#include <uipc/constitution/particle.h>
#include <uipc/constitution/soft_vertex_edge_stitch.h>

TEST_CASE("68_soft_vertex_edge_stitch", "[inter_primitive]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    Particle particle;
    Empty    empty;

    vector<Vector3> vert_pos = {
        {0.25, 0.5, 0.0}, {0.5, 0.3, 0.0}, {0.75, 0.4, 0.0}, {0.5, 0.0, 0.0}};
    auto vertex_mesh = pointcloud(vert_pos);
    particle.apply_to(vertex_mesh);
    label_surface(vertex_mesh);

    // Edge in XOZ plane (y=0)
    vector<Vector3>  edge_pos  = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    vector<Vector2i> edge_segs = {{0, 1}};
    auto             edge_mesh = linemesh(edge_pos, edge_segs);
    empty.apply_to(edge_mesh);
    label_surface(edge_mesh);
    auto is_fixed = edge_mesh.vertices().find<IndexT>(builtin::is_fixed);
    std::ranges::fill(view(*is_fixed), 1);

    auto vert_obj = scene.objects().create("vertex_provider");
    auto [vert_geo_slot, vert_rest_slot] = vert_obj->geometries().create(vertex_mesh);

    auto edge_obj = scene.objects().create("edge_provider");
    auto [edge_geo_slot, edge_rest_slot] = edge_obj->geometries().create(edge_mesh);

    auto pairs_geo = closest_vertex_edge_pairs(vertex_mesh, edge_mesh, 1.0);
    REQUIRE(pairs_geo.instances().size() >= 1);

    SoftVertexEdgeStitch stitch;
    auto                 stitch_geo =
        stitch.create_geometry({vert_geo_slot, edge_geo_slot},
                               {vert_rest_slot, edge_rest_slot},
                               pairs_geo,
                               ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
                               0.1,
                               0.1);

    auto stitch_obj = scene.objects().create("stitch");
    stitch_obj->geometries().create(stitch_geo);

    world.init(scene);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    REQUIRE(world.is_valid());

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
