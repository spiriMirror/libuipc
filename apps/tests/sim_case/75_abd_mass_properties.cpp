#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>

TEST_CASE("75_abd_mass_properties", "[abd][mass]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::core;
    using namespace uipc::constitution;
    namespace fs     = std::filesystem;
    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = true;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    scene.contact_tabular().default_model(0.5, 1.0_GPa);
    auto default_contact = scene.contact_tabular().default_element();

    AffineBodyConstitution abd;

    // --- Test 1: standard path (mass_density) ---
    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};
    vector<Vector3>  Vs = {Vector3{0, 1, 0},
                           Vector3{0, 0, 1},
                           Vector3{-std::sqrt(3) / 2, 0, -0.5},
                           Vector3{std::sqrt(3) / 2, 0, -0.5}};
    std::transform(Vs.begin(), Vs.end(), Vs.begin(), [](auto& v) { return v * 0.3; });

    auto mesh1 = tetmesh(Vs, Ts);
    label_surface(mesh1);
    label_triangle_orient(mesh1);

    Float rho = 1000.0;
    abd.apply_to(mesh1, 100.0_MPa, rho);

    {
        auto mass_attr = mesh1.meta().find<Float>("mass");
        REQUIRE(mass_attr);
        Float mass_val = mass_attr->view()[0];
        REQUIRE(mass_val > 0);

        auto mc_attr = mesh1.meta().find<Vector3>("mass_center");
        REQUIRE(mc_attr);

        auto inertia_attr = mesh1.meta().find<Matrix3x3>("inertia");
        REQUIRE(inertia_attr);

        auto abd_m_attr = mesh1.meta().find<Float>(builtin::abd_mass);
        REQUIRE(abd_m_attr);
        REQUIRE(abd_m_attr->view()[0] == Catch::Approx(mass_val));

        spdlog::info("Standard path: mass={}, center=({},{},{})",
                     mass_val,
                     mc_attr->view()[0].x(),
                     mc_attr->view()[0].y(),
                     mc_attr->view()[0].z());
    }

    // --- Test 2: Matrix12x12 override path ---
    auto mesh2 = tetmesh(Vs, Ts);
    label_surface(mesh2);
    label_triangle_orient(mesh2);

    Float     input_mass = 5.0;
    Vector3   input_com  = Vector3{0.1, 0.2, 0.3};
    Matrix3x3 input_I    = Matrix3x3::Identity() * 0.01;
    auto M12 = affine_body::from_rigid_body(input_mass, input_com, input_I);

    Float override_volume = 0.001;
    abd.apply_to(mesh2, 100.0_MPa, M12, override_volume);

    {
        auto mass_attr = mesh2.meta().find<Float>("mass");
        REQUIRE(mass_attr);
        REQUIRE(mass_attr->view()[0] == Catch::Approx(input_mass).epsilon(1e-10));

        auto mc_attr = mesh2.meta().find<Vector3>("mass_center");
        REQUIRE(mc_attr);
        auto com = mc_attr->view()[0];
        REQUIRE(com.x() == Catch::Approx(input_com.x()).epsilon(1e-10));
        REQUIRE(com.y() == Catch::Approx(input_com.y()).epsilon(1e-10));
        REQUIRE(com.z() == Catch::Approx(input_com.z()).epsilon(1e-10));

        auto inertia_attr = mesh2.meta().find<Matrix3x3>("inertia");
        REQUIRE(inertia_attr);
        auto I_out = inertia_attr->view()[0];
        for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 3; ++c)
                REQUIRE(I_out(r, c) == Catch::Approx(input_I(r, c)).epsilon(1e-8));

        spdlog::info("Override path: mass={}, center=({},{},{})",
                     mass_attr->view()[0],
                     com.x(), com.y(), com.z());
    }

    // --- Run a short simulation to verify backend reads abd_mass correctly ---
    auto object = scene.objects().create("test_body");
    default_contact.apply_to(mesh1);
    mesh1.instances().resize(1);
    auto trans_view = view(mesh1.transforms());
    Transform t     = Transform::Identity();
    t.translate(Vector3{0, 1, 0});
    trans_view[0] = t.matrix();
    object->geometries().create(mesh1);

    world.init(scene);
    REQUIRE(world.is_valid());

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", output_path, 0));

    while(world.frame() < 3)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", output_path, world.frame()));
    }
}
