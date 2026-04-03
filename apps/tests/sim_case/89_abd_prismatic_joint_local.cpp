#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>

TEST_CASE("89_abd_prismatic_joint_local", "[abd][joint][local]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                            = test::Scene::default_config();
    config["gravity"]                      = Vector3{0, 0, 0};
    config["contact"]["enable"]            = false;
    config["line_search"]["report_energy"] = true;
    config["dt"]                           = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};

    AffineBodyConstitution abd;

    // Aligned pair (z = -0.5)
    // Bodies adjacent, prismatic axis along X, both share the same axis line
    auto al_left_obj = scene.objects().create("aligned_left");
    SimplicialComplex al_left_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(al_left_mesh, 100.0_MPa);
    label_surface(al_left_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{-0.15, 0.5, -0.5});
        view(al_left_mesh.transforms())[0] = t.matrix();
        auto is_fixed = al_left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;
    }
    auto [al_left_slot, _al] = al_left_obj->geometries().create(al_left_mesh);

    auto al_right_obj = scene.objects().create("aligned_right");
    SimplicialComplex al_right_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(al_right_mesh, 100.0_MPa);
    label_surface(al_right_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{0.15, 0.5, -0.5});
        view(al_right_mesh.transforms())[0] = t.matrix();
        auto is_dynamic = al_right_mesh.instances().find<IndexT>(builtin::is_dynamic);
        view(*is_dynamic)[0] = 0;
    }
    auto [al_right_slot, _ar] = al_right_obj->geometries().create(al_right_mesh);

    // Misaligned pair (z = +0.5)
    // Left body at same layout, right body slightly offset
    auto mis_left_obj = scene.objects().create("misaligned_left");
    SimplicialComplex mis_left_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(mis_left_mesh, 100.0_MPa);
    label_surface(mis_left_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{-0.15, 0.5, 0.5});
        view(mis_left_mesh.transforms())[0] = t.matrix();
        auto is_fixed = mis_left_mesh.instances().find<IndexT>(builtin::is_fixed);
        view(*is_fixed)[0] = 1;
    }
    auto [mis_left_slot, _ml] = mis_left_obj->geometries().create(mis_left_mesh);

    auto mis_right_obj = scene.objects().create("misaligned_right");
    SimplicialComplex mis_right_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(mis_right_mesh, 100.0_MPa);
    label_surface(mis_right_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{0.2, 0.55, 0.55});
        view(mis_right_mesh.transforms())[0] = t.matrix();
        auto is_dynamic = mis_right_mesh.instances().find<IndexT>(builtin::is_dynamic);
        view(*is_dynamic)[0] = 0;
    }
    auto [mis_right_slot, _mr] = mis_right_obj->geometries().create(mis_right_mesh);

    // Two prismatic joints with identical local attachment points
    AffineBodyPrismaticJoint prismatic_joint;

    vector<Vector3> l_pos0 = {Vector3{-0.15, 0, 0}, Vector3{-0.15, 0, 0}};
    vector<Vector3> l_pos1 = {Vector3{ 0.15, 0, 0}, Vector3{ 0.15, 0, 0}};
    vector<Vector3> r_pos0 = {Vector3{-0.15, 0, 0}, Vector3{-0.15, 0, 0}};
    vector<Vector3> r_pos1 = {Vector3{ 0.15, 0, 0}, Vector3{ 0.15, 0, 0}};

    vector<S<SimplicialComplexSlot>> l_geo_slots    = {al_left_slot, mis_left_slot};
    vector<IndexT>                   l_instance_ids = {0, 0};
    vector<S<SimplicialComplexSlot>> r_geo_slots    = {al_right_slot, mis_right_slot};
    vector<IndexT>                   r_instance_ids = {0, 0};
    vector<Float>                    strength_ratios = {100.0, 100.0};

    auto joint_mesh = prismatic_joint.create_geometry(
        span{l_pos0}, span{l_pos1}, span{r_pos0}, span{r_pos1},
        span{l_geo_slots}, span{l_instance_ids},
        span{r_geo_slots}, span{r_instance_ids},
        span{strength_ratios});

    auto joint_object = scene.objects().create("prismatic_joint");
    joint_object->geometries().create(joint_mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 5)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }

    auto to_world = [](const Matrix4x4& T, const Vector3& p) -> Vector3
    {
        return (T * Vector4{p.x(), p.y(), p.z(), 1.0}).head<3>();
    };

    auto al_left_t   = view(al_left_slot->geometry().as<SimplicialComplex>()->transforms())[0];
    auto al_right_t  = view(al_right_slot->geometry().as<SimplicialComplex>()->transforms())[0];
    auto mis_left_t  = view(mis_left_slot->geometry().as<SimplicialComplex>()->transforms())[0];
    auto mis_right_t = view(mis_right_slot->geometry().as<SimplicialComplex>()->transforms())[0];

    Vector3 lp0{-0.15, 0, 0};
    Vector3 lp1{ 0.15, 0, 0};
    Vector3 rp0{-0.15, 0, 0};
    Vector3 rp1{ 0.15, 0, 0};

    auto check_prismatic = [&](const Matrix4x4& lt, const Matrix4x4& rt)
    {
        Vector3 l_axis = (to_world(lt, lp1) - to_world(lt, lp0)).normalized();
        Vector3 r_axis = (to_world(rt, rp1) - to_world(rt, rp0)).normalized();
        REQUIRE(l_axis.cross(r_axis).isZero());

        Vector3 offset = to_world(rt, rp0) - to_world(lt, lp0);
        Vector3 perp   = offset - offset.dot(l_axis) * l_axis;
        REQUIRE(perp.isZero());
    };

    check_prismatic(al_left_t, al_right_t);
    check_prismatic(mis_left_t, mis_right_t);
}
