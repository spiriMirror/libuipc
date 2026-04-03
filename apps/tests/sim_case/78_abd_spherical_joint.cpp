#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_spherical_joint.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <numbers>

TEST_CASE("78_abd_spherical_joint", "[abd][joint]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                            = test::Scene::default_config();
    config["gravity"]                      = Vector3{0, -9.8, 0};
    config["contact"]["enable"]            = false;
    config["line_search"]["report_energy"] = true;
    config["dt"]                           = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};

    AffineBodyConstitution  abd;
    SoftTransformConstraint stc;

    // ---- Left body: animated via SoftTransformConstraint ----
    auto              left_object = scene.objects().create("left");
    SimplicialComplex left_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(left_mesh, 100.0_MPa);
    stc.apply_to(left_mesh, Vector2{100, 100});
    label_surface(left_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{0, 0.5, 0});
        view(left_mesh.transforms())[0] = t.matrix();
        auto is_fixed = left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;
    }
    auto [left_geo_slot, _l] = left_object->geometries().create(left_mesh);

    // ---- Right body: free, connected to left via spherical joint ----
    auto              right_object = scene.objects().create("right");
    SimplicialComplex right_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(right_mesh, 100.0_MPa);
    label_surface(right_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{0.15, 0.2, 0.15});
        view(right_mesh.transforms())[0] = t.matrix();
    }
    auto [right_geo_slot, _r] = right_object->geometries().create(right_mesh);

    // ---- Spherical joint ----
    // Right body center = {0.15, 0.2, 0.15}; anchor in right body local = {-0.15, 0.15, -0.15}
    // World anchor = body_translation + local_anchor = {0, 0.35, 0}
    AffineBodySphericalJoint         spherical_joint;
    vector<Vector3>                  positions      = {Vector3{0, 0.35, 0}};
    vector<S<SimplicialComplexSlot>> l_geo_slots    = {left_geo_slot};
    vector<IndexT>                   l_instance_ids = {0};
    vector<S<SimplicialComplexSlot>> r_geo_slots    = {right_geo_slot};
    vector<IndexT>                   r_instance_ids = {0};
    vector<Float>                    strength_ratios = {100.0};
    auto joint_mesh = spherical_joint.create_geometry(
        span{positions},
        span{l_geo_slots}, span{l_instance_ids},
        span{r_geo_slots}, span{r_instance_ids},
        span{strength_ratios});

    auto joint_object = scene.objects().create("spherical_joint");
    joint_object->geometries().create(joint_mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 50)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
