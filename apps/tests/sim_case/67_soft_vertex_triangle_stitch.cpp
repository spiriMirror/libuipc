#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/empty.h>
#include <uipc/constitution/particle.h>
#include <uipc/constitution/soft_vertex_triangle_stitch.h>

TEST_CASE("67_soft_vertex_triangle_stitch", "[inter_primitive]")
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

    Float Y = 1.0;

    vector<Vector3> vert_pos = {
        {0.25, Y - 0.5, 0.25}, {0.5, Y - 0.3, 0.1}, {0.1, Y - 0.4, 0.5}, {0.3, Y, 0.3}};
    auto vertex_mesh = pointcloud(vert_pos);
    particle.apply_to(vertex_mesh);
    label_surface(vertex_mesh);

    // Triangle in XOZ plane (y=1.0)
    vector<Vector3> tri_pos = {{0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 1.0}};
    vector<Vector3i> tri_faces = {{0, 1, 2}};

    auto triangle_mesh = trimesh(tri_pos, tri_faces);
    empty.apply_to(triangle_mesh);
    label_surface(triangle_mesh);
    auto is_fixed = triangle_mesh.vertices().find<IndexT>(builtin::is_fixed);
    std::ranges::fill(view(*is_fixed), 1);

    auto vert_obj = scene.objects().create("vertex_provider");
    auto [vert_geo_slot, vert_rest_slot] = vert_obj->geometries().create(vertex_mesh);

    auto tri_obj = scene.objects().create("triangle_provider");
    auto [tri_geo_slot, tri_rest_slot] = tri_obj->geometries().create(triangle_mesh);

    auto pairs_geo = closest_vertex_triangle_pairs(vertex_mesh, triangle_mesh, 1.0);
    REQUIRE(pairs_geo.instances().size() >= 1);

    SoftVertexTriangleStitch stitch;
    auto                     stitch_geo =
        stitch.create_geometry({vert_geo_slot, tri_geo_slot},
                               {vert_rest_slot, tri_rest_slot},
                               pairs_geo,
                               ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
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
