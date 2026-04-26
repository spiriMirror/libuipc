#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>
#include <numbers>

TEST_CASE("92_abd_stc_off_origin_cm", "[animation][abd]")
{
    // Test: SoftTransformConstraint with translation-only strength (eta_p=100, eta_a=0)
    // must drive the body's center of mass toward the aim's CM position,
    // even when the CM is not at the model-space origin (c != 0).

    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                         = test::Scene::default_config();
    config["gravity"]                   = Vector3{0, 0, 0};
    config["contact"]["enable"]         = false;
    config["line_search"]["max_iter"]   = 8;
    config["linear_system"]["tol_rate"] = 1e-3;
    config["dt"]                        = 0.01;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    AffineBodyConstitution  abd;
    SoftTransformConstraint stc;

    // Simple tet mesh — geometry shape does not matter since mass is overridden
    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
    vector<Vector3>  Vs = {Vector3{0, 1, 0},
                           Vector3{0, 0, 1},
                           Vector3{-std::sqrt(3) / 2, 0, -0.5},
                           Vector3{std::sqrt(3) / 2, 0, -0.5}};

    auto mesh = tetmesh(Vs, Ts);
    label_surface(mesh);
    label_triangle_orient(mesh);

    // Override mass properties: m = 1 kg, CM at (0, 1, 0) in model space
    Vector3   model_com  = Vector3{0.0, 1.0, 0.0};
    Float     body_mass  = 1.0;
    Matrix3x3 inertia_cm = Matrix3x3::Identity();
    auto      M12        = affine_body::from_rigid_body(body_mass, model_com, inertia_cm);

    // kappa = 100 MPa keeps A ~ I throughout (body behaves rigidly)
    abd.apply_to(mesh, 100.0_MPa, M12, 0.001);
    // Translation-only soft constraint: eta_p = 100, eta_a = 0
    stc.apply_to(mesh, Vector2{100.0, 0.0});

    auto object            = scene.objects().create("body");
    auto [geo_slot, _body] = object->geometries().create(mesh);

    // Animator: ramp the aim from Identity to Rz(90 deg) over 180 frames.
    // The aim has p_aim = 0 and A_aim = Rz(theta), so the aim CM is Rz(theta)*c.
    constexpr Float pi       = std::numbers::pi;
    constexpr int   ramp_end = 50;

    auto& animator = scene.animator();
    animator.insert(
        *object,
        [](Animation::UpdateInfo& info)
        {
            constexpr Float pi_val   = std::numbers::pi;
            constexpr int   ramp_max = 50;

            auto geo = info.geo_slots()[0]->geometry().as<SimplicialComplex>();

            auto is_constrained = geo->instances().find<IndexT>(builtin::is_constrained);
            view(*is_constrained)[0] = 1;

            auto aim      = geo->instances().find<Matrix4x4>(builtin::aim_transform);
            auto aim_view = view(*aim);

            int   t     = std::min((int)info.frame(), ramp_max);
            Float theta = (pi_val / 2.0) * (Float(t) / Float(ramp_max));

            Transform transform = Transform::Identity();
            transform.rotate(Eigen::AngleAxisd(theta, Vector3::UnitZ()));
            aim_view[0] = transform.matrix();
        });

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    // Run ramp_end frames plus 20 extra for convergence at the final aim
    while(world.frame() < ramp_end + 20)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }

    // Read back the final body transform and compute the world-space CM position:
    //   x_cm = p + A * model_com
    // The 4x4 transform matrix layout (from q_to_transform):
    //   T[i,j] = A[i,j] for i,j in {0,1,2},   T[i,3] = p[i]
    auto      T    = view(geo_slot->geometry().as<SimplicialComplex>()->transforms())[0];
    Vector3   p    = T.block<3, 1>(0, 3);
    Matrix3x3 A    = T.block<3, 3>(0, 0);
    Vector3   x_cm = p + A * model_com;

    // Target CM = Rz(90 deg) * (0,1,0) = (-1, 0, 0)
    Vector3 cm_target = Vector3{-1.0, 0.0, 0.0};

    REQUIRE(x_cm.isApprox(cm_target));
}
