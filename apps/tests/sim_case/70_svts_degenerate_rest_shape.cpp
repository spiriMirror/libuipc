#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_shell.h>
#include <uipc/constitution/particle.h>
#include <uipc/constitution/soft_vertex_triangle_stitch.h>

TEST_CASE("70_svts_degenerate_rest_shape", "[inter_primitive]")
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

    Particle         particle;
    AffineBodyShell  abd_shell;

    // Triangle: mesh-local positions centered at origin, translated via instance transform.
    // Uses AffineBodyShell so instance transforms are handled natively.
    Float              S         = 0.5;
    Float              Y_offset  = 1.0;
    vector<Vector3>    tri_pos   = {{-S, 0, -S}, {S, 0, -S}, {-S, 0, S}};
    vector<Vector3i>   tri_faces = {{0, 1, 2}};

    auto triangle_mesh = trimesh(tri_pos, tri_faces);
    abd_shell.apply_to(triangle_mesh, 100.0_MPa);
    label_surface(triangle_mesh);
    auto is_fixed = triangle_mesh.instances().find<IndexT>(builtin::is_fixed);
    std::ranges::fill(view(*is_fixed), 1);

    // Apply instance transform: translate triangle up to y=1.0
    auto      tri_trans_view = view(triangle_mesh.transforms());
    Transform tri_t          = Transform::Identity();
    tri_t.translate(Vector3{0.0, Y_offset, 0.0});
    tri_trans_view[0] = tri_t.matrix();

    // Particle: world-space position on the triangle plane at (0, Y_offset, 0)
    vector<Vector3> vert_pos = {{0.0, Y_offset, 0.0}};
    auto            vertex_mesh = pointcloud(vert_pos);
    particle.apply_to(vertex_mesh);
    label_surface(vertex_mesh);

    auto vert_obj = scene.objects().create("vertex_provider");
    auto [vert_geo_slot, vert_rest_slot] =
        vert_obj->geometries().create(vertex_mesh);

    auto tri_obj = scene.objects().create("triangle_provider");
    auto [tri_geo_slot, tri_rest_slot] =
        tri_obj->geometries().create(triangle_mesh);

    // closest_vertex_triangle_pairs needs world-space meshes
    auto tri_world = apply_transform(triangle_mesh);
    REQUIRE(tri_world.size() == 1);
    auto pairs_geo = closest_vertex_triangle_pairs(vertex_mesh, tri_world[0], 0.01);
    REQUIRE(pairs_geo.instances().size() == 1);

    SoftVertexTriangleStitch stitch;
    auto stitch_geo = stitch.create_geometry({vert_geo_slot, tri_geo_slot},
                                             {vert_rest_slot, tri_rest_slot},
                                             pairs_geo,
                                             ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
                                             0.05);

    auto stitch_obj = scene.objects().create("stitch");
    stitch_obj->geometries().create(stitch_geo);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    Vector3 initial_pos = vert_pos[0];

    while(world.frame() < 10)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

        auto geo = vert_geo_slot->geometry().as<SimplicialComplex>();
        auto pos_view = geo->positions().view();
        Vector3 current_pos = pos_view[0];

        Float lateral_drift = std::sqrt(current_pos.x() * current_pos.x()
                                        + current_pos.z() * current_pos.z());

        Float normal_drift = std::abs(current_pos.y() - initial_pos.y());

        INFO(fmt::format("Frame {}: lateral_drift={:.6f}, normal_drift={:.6f}, "
                         "pos=[{:.4f},{:.4f},{:.4f}]",
                         world.frame(),
                         lateral_drift,
                         normal_drift,
                         current_pos.x(),
                         current_pos.y(),
                         current_pos.z()));

        REQUIRE(lateral_drift < 0.001);
        REQUIRE(normal_drift < 0.1);
    }
}
