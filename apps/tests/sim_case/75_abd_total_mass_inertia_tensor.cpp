#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <uipc/builtin/attribute_name.h>

TEST_CASE("75_abd_total_mass_inertia_tensor", "[abd]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    using namespace uipc::builtin;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    Float   dt = 0.01;
    Vector3 gravity{0, -9.8, 0};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = gravity;
    config["contact"]["enable"] = false;
    config["dt"]                = dt;
    test::Scene::dump_config(config, output_path);

    Float volume = 1.0;

    Scene scene{config};

    scene.contact_tabular().default_model(0.5, 1.0_GPa);

    SimplicialComplexIO io;
    SimplicialComplex   mesh =
        io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    label_surface(mesh);

    AffineBodyConstitution abd;
    auto                   object = scene.objects().create("abd");

    Float       mass = 1000.0;
    Vector3     com  = Vector3::Zero();
    Matrix3x3   I_cm = (mass / 12.0) * Matrix3x3::Identity() * 2;
    Matrix12x12 M    = affine_body::from_rigid_body(mass, com, I_cm);

    mesh.instances().resize(1);
    abd.apply_to(mesh, 100.0_MPa, M, volume);

    auto trans_view = view(mesh.transforms());
    auto is_fixed_view = view(*mesh.instances().find<IndexT>(builtin::is_fixed));

    auto [cube_geo_slot, _] = object->geometries().create(mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, world.frame()));

    {
        // first method: use the geometry slot
        auto geo = cube_geo_slot->geometry().as<SimplicialComplex>();
        auto total_mass_view = view(*geo->instances().find<Float>(builtin::total_mass));
        auto inertia_view =
            view(*geo->instances().find<Matrix3x3>(builtin::inertia_tensor));
        REQUIRE(total_mass_view.size() == 1);
        REQUIRE(total_mass_view[0] > 0);
        REQUIRE(total_mass_view[0] == Catch::Approx(mass));

        REQUIRE(inertia_view.size() == 1);
        const Matrix3x3& I = inertia_view[0];
        REQUIRE((I - I.transpose()).norm() == Catch::Approx(0.0));
        REQUIRE(I(0, 0) == Catch::Approx(I_cm(0, 0)));
        REQUIRE(I(1, 1) == Catch::Approx(I_cm(1, 1)));
        REQUIRE(I(2, 2) == Catch::Approx(I_cm(2, 2)));
    }

    // second method: use the affine body state accessor feature
    {
        auto abd_accessor = world.features().find<AffineBodyStateAccessorFeature>();
        REQUIRE(abd_accessor != nullptr);
        REQUIRE(abd_accessor->body_count() == 1);

        SimplicialComplex abd_state = abd_accessor->create_geometry();
        abd_state.instances().create<Matrix4x4>(builtin::transform);
        abd_state.instances().create<Float>(builtin::total_mass);
        abd_state.instances().create<Matrix3x3>(builtin::inertia_tensor);
        abd_accessor->copy_to(abd_state);

        auto total_mass_view =
            view(*abd_state.instances().find<Float>(builtin::total_mass));
        auto inertia_view =
            view(*abd_state.instances().find<Matrix3x3>(builtin::inertia_tensor));
        REQUIRE(total_mass_view.size() == 1);
        REQUIRE(total_mass_view[0] > 0);
        REQUIRE(total_mass_view[0] == Catch::Approx(mass));

        REQUIRE(inertia_view.size() == 1);
        const Matrix3x3& I = inertia_view[0];
        REQUIRE((I - I.transpose()).norm() == Catch::Approx(0.0));
        REQUIRE(I(0, 0) == Catch::Approx(I_cm(0, 0)));
        REQUIRE(I(1, 1) == Catch::Approx(I_cm(1, 1)));
        REQUIRE(I(2, 2) == Catch::Approx(I_cm(2, 2)));
    }
}
