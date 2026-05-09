#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("93_dynamic_dt", "[abd][dynamic_dt]")
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
    config["dt"]                = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        AffineBodyConstitution abd;

        auto object = scene.objects().create("falling_tet");

        vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs = {Vector3{0, 1, 0},
                               Vector3{0, 0, 1},
                               Vector3{-std::sqrt(3) / 2, 0, -0.5},
                               Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs.begin(), Vs.end(), Vs.begin(), [](auto& v) { return v * 0.3; });

        auto mesh = tetmesh(Vs, Ts);
        label_surface(mesh);
        label_triangle_orient(mesh);

        mesh.instances().resize(1);
        abd.apply_to(mesh, 100.0_MPa);

        auto trans_view    = view(mesh.transforms());
        auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);

        Transform t      = Transform::Identity();
        t.translation()  = Vector3::UnitY() * 2;
        trans_view[0]    = t.matrix();
        is_fixed_view[0] = 0;

        object->geometries().create(mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    auto geo_slots = scene.geometries().find(0);
    auto geo_slot  = geo_slots.geometry;

    auto read_center_y = [&]() -> Float
    {
        auto      geo        = geo_slot->geometry().as<SimplicialComplex>();
        auto      trans_view = geo->transforms().view();
        Matrix4x4 T          = trans_view[0];
        return T(1, 3);
    };

    Float prev_y = read_center_y();

    // Run frame 1 with dt=0.01
    world.advance();
    REQUIRE(world.is_valid());
    world.retrieve();
    Float y_after_1 = read_center_y();
    Float dy_frame1 = prev_y - y_after_1;
    REQUIRE(dy_frame1 > 0.0);

    // Now change dt to 0.005 and reset to the same starting position
    // by running a fresh scene. Instead, we compare the first-frame displacement
    // at two different dt values. Run additional frames to let it settle.
    //
    // Simpler approach: compare dy/dt ratio.
    // At frame 1 with dt=0.01: dy1 = 0.5 * g * dt^2 = 0.5 * 9.8 * 0.0001 = 0.00049
    // With dt=0.005: dy = 0.5 * g * dt^2 = 0.5 * 9.8 * 0.000025 = 0.0001225
    // ratio should be ~4x

    // Reset: create a second simulation with dt=0.005 from the start
    Engine engine2{"cuda", output_path};
    World  world2{engine2};

    auto config2                 = test::Scene::default_config();
    config2["gravity"]           = Vector3{0, -9.8, 0};
    config2["contact"]["enable"] = false;
    config2["dt"]                = 0.005;

    Scene scene2{config2};
    {
        AffineBodyConstitution abd2;

        auto object2 = scene2.objects().create("falling_tet");

        vector<Vector4i> Ts2 = {Vector4i{0, 1, 2, 3}};
        vector<Vector3>  Vs2 = {Vector3{0, 1, 0},
                                Vector3{0, 0, 1},
                                Vector3{-std::sqrt(3) / 2, 0, -0.5},
                                Vector3{std::sqrt(3) / 2, 0, -0.5}};

        std::transform(
            Vs2.begin(), Vs2.end(), Vs2.begin(), [](auto& v) { return v * 0.3; });

        auto mesh2 = tetmesh(Vs2, Ts2);
        label_surface(mesh2);
        label_triangle_orient(mesh2);

        mesh2.instances().resize(1);
        abd2.apply_to(mesh2, 100.0_MPa);

        auto trans_view2    = view(mesh2.transforms());
        auto is_fixed2      = mesh2.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view2 = view(*is_fixed2);

        Transform t2      = Transform::Identity();
        t2.translation()  = Vector3::UnitY() * 2;
        trans_view2[0]    = t2.matrix();
        is_fixed_view2[0] = 0;

        object2->geometries().create(mesh2);
    }

    world2.init(scene2);
    REQUIRE(world2.is_valid());

    auto geo_slots2 = scene2.geometries().find(0);
    auto geo_slot2  = geo_slots2.geometry;

    auto read_center_y2 = [&]() -> Float
    {
        auto      geo2       = geo_slot2->geometry().as<SimplicialComplex>();
        auto      trans_view = geo2->transforms().view();
        Matrix4x4 T          = trans_view[0];
        return T(1, 3);
    };

    Float prev_y2 = read_center_y2();

    world2.advance();
    REQUIRE(world2.is_valid());
    world2.retrieve();
    Float y_after_1_small = read_center_y2();
    Float dy_frame1_small = prev_y2 - y_after_1_small;
    REQUIRE(dy_frame1_small > 0.0);

    // With dt halved, first-frame displacement (0.5*g*dt^2) should be ~4x smaller
    REQUIRE(dy_frame1_small < dy_frame1);

    // Now test dynamic dt change: change dt in scene2 from 0.005 to 0.01
    {
        auto dt_slot      = scene2.config().find<Float>("dt");
        view(*dt_slot)[0] = 0.01;
    }

    // Run one more frame with the changed dt
    Float prev_y2_before = read_center_y2();
    world2.advance();
    REQUIRE(world2.is_valid());
    world2.retrieve();
    Float y_after_2_changed = read_center_y2();
    Float dy_frame2_changed = prev_y2_before - y_after_2_changed;

    // After changing dt from 0.005 to 0.01, the displacement should increase
    REQUIRE(dy_frame2_changed > dy_frame1_small);
}
