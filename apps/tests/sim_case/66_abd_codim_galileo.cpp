#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_shell.h>
#include <uipc/constitution/affine_body_rod.h>
#include <uipc/core/affine_body_state_accessor_feature.h>

TEST_CASE("66_abd_codim_galileo", "[abd][codim]")
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
    {
        AffineBodyConstitution abd_3d;
        AffineBodyShell        abd_shell;
        AffineBodyRod          abd_rod;

        {
            auto obj = scene.objects().create("tet_3d");

            Float h = 1.0 / std::sqrt(2.0);
            vector<Vector3> Vs = {
                Vector3{-1,  0, -h},
                Vector3{ 1,  0, -h},
                Vector3{ 0,  1,  h},
                Vector3{ 0, -1,  h}};
            std::transform(Vs.begin(), Vs.end(), Vs.begin(),
                           [](const Vector3& v) { return v * 0.3; });

            vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

            auto mesh = tetmesh(Vs, Ts);
            label_surface(mesh);
            label_triangle_orient(mesh);

            abd_3d.apply_to(mesh, 100.0_MPa);

            auto trans_view    = view(mesh.transforms());
            Transform t        = Transform::Identity();
            t.translation()    = Vector3{-1.5, 1.0, 0};
            trans_view[0]      = t.matrix();

            obj->geometries().create(mesh);
        }

        {
            auto obj = scene.objects().create("shell");

            vector<Vector3> Vs = {
                Vector3{-0.3, 0, -0.3},
                Vector3{ 0.3, 0, -0.3},
                Vector3{ 0.3, 0,  0.3},
                Vector3{-0.3, 0,  0.3}};

            vector<Vector3i> Fs = {
                Vector3i{0, 1, 2},
                Vector3i{0, 2, 3}};

            auto mesh = trimesh(Vs, Fs);
            label_surface(mesh);

            abd_shell.apply_to(mesh, 100.0_MPa, 1e3, 0.02);

            auto trans_view    = view(mesh.transforms());
            Transform t        = Transform::Identity();
            t.translation()    = Vector3{0, 1.0, 0};
            trans_view[0]      = t.matrix();

            obj->geometries().create(mesh);
        }

        {
            auto obj = scene.objects().create("rod");

            vector<Vector3>  Vs = {Vector3{-0.3, 0, 0}, Vector3{0.3, 0, 0}};
            vector<Vector2i> Es = {Vector2i{0, 1}};

            auto mesh = linemesh(Vs, Es);
            label_surface(mesh);

            abd_rod.apply_to(mesh, 100.0_MPa, 1e3, 0.02);

            auto trans_view    = view(mesh.transforms());
            Transform t        = Transform::Identity();
            t.translation()    = Vector3{1.5, 1.0, 0};
            trans_view[0]      = t.matrix();

            obj->geometries().create(mesh);
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    // Capture ABD transforms to verify the Galileo free-fall property:
    // all three body types (3D tet, shell, rod) must fall at the same rate.
    // Body order: 0=tet_3d, 1=shell, 2=rod  (creation order above)
    auto abd_accessor = world.features().find<AffineBodyStateAccessorFeature>();
    REQUIRE(abd_accessor != nullptr);
    SimplicialComplex abd_state = abd_accessor->create_geometry();
    abd_state.instances().create<Matrix4x4>(builtin::transform);

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 50)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

        // At frame 20 verify all three bodies have the same y-translation.
        // The translation is the (row=1, col=3) entry of the 4x4 affine matrix.
        if(world.frame() == 20)
        {
            abd_accessor->copy_to(abd_state);
            auto trans_view = abd_state.transforms().view();
            REQUIRE(trans_view.size() == 3);

            Float y_tet   = trans_view[0](1, 3);
            Float y_shell = trans_view[1](1, 3);
            Float y_rod   = trans_view[2](1, 3);

            // All bodies started at y = 1.0 and should have fallen the same amount.
            REQUIRE(y_tet < 1.0);
            REQUIRE(y_tet == Catch::Approx(y_shell).epsilon(1e-3));
            REQUIRE(y_tet == Catch::Approx(y_rod).epsilon(1e-3));
        }
    }
}

