#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_fixed_joint.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <numbers>

TEST_CASE("79_abd_fixed_joint", "[abd][joint]")
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
        t.translate(Vector3{-0.5, 0.5, 0});
        view(left_mesh.transforms())[0] = t.matrix();
    }
    auto [left_geo_slot, _l] = left_object->geometries().create(left_mesh);

    // ---- Right body: free, connected to left via fixed joint ----
    auto              right_object = scene.objects().create("right");
    SimplicialComplex right_mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    abd.apply_to(right_mesh, 100.0_MPa);
    label_surface(right_mesh);
    {
        Transform t = Transform::Identity();
        t.translate(Vector3{0.5, 0.5, 0});
        view(right_mesh.transforms())[0] = t.matrix();
    }
    auto [right_geo_slot, _r] = right_object->geometries().create(right_mesh);

    // ---- Fixed joint ----
    SimplicialComplex                joint_mesh;
    AffineBodyFixedJoint             fixed_joint;
    vector<S<SimplicialComplexSlot>> l_geo_slots = {left_geo_slot};
    vector<S<SimplicialComplexSlot>> r_geo_slots = {right_geo_slot};
    fixed_joint.apply_to(joint_mesh, span{l_geo_slots}, span{r_geo_slots}, 100.0);

    auto joint_object = scene.objects().create("fixed_joint");
    joint_object->geometries().create(joint_mesh);

    // ---- Animator: move the left body, right body should follow ----
    auto& animator = scene.animator();
    animator.insert(
        *left_object,
        [](Animation::UpdateInfo& info)
        {
            auto geo_slots = info.geo_slots();
            auto geo       = geo_slots[0]->geometry().as<SimplicialComplex>();

            auto is_constrained = geo->instances().find<IndexT>(builtin::is_constrained);
            view(*is_constrained)[0] = 1;

            auto aim     = geo->instances().find<Matrix4x4>(builtin::aim_transform);
            auto aim_view = view(*aim);

            constexpr Float pi = std::numbers::pi;
            Float           t  = info.frame() * info.dt();

            Float x_offset = 0.5 * std::sin(2 * pi * t);
            Float y_offset = 0.3 * std::sin(4 * pi * t);
            Float theta    = pi / 4 * std::sin(2 * pi * t);

            Transform transform = Transform::Identity();
            transform.translate(Vector3{-0.5 + x_offset, 0.5 + y_offset, 0});
            transform.rotate(Eigen::AngleAxisd(theta, Vector3::UnitZ()));
            aim_view[0] = transform.matrix();
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    while(world.frame() < 200)
    {
        world.advance();
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
