#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>

TEST_CASE("84_proxy_abd_gravity", "[abd]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    auto output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    std::string tetmesh_dir{AssetDir::tetmesh_path()};

    Engine engine{"cuda", output_path};
    World  world{engine};

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};

    AffineBodyConstitution abd;

    Float     kappa = 100.0_MPa;
    Float     rho   = 1e3;
    Vector3   start = Vector3::UnitY();
    Transform t0    = Transform::Identity();
    t0.translation() = start;

    // --- Body A: full ABD cube ---
    auto cube_object = scene.objects().create("cube");
    Transform pre_transform = Transform::Identity();
    pre_transform.scale(0.3);
    SimplicialComplexIO io{pre_transform};
    auto cube_mesh = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));
    label_surface(cube_mesh);
    label_triangle_orient(cube_mesh);
    abd.apply_to(cube_mesh, kappa, rho);

    view(cube_mesh.transforms())[0] = t0.matrix();

    auto cube_mass    = cube_mesh.meta().find<Float>("mass")->view()[0];
    auto cube_com     = cube_mesh.meta().find<Vector3>("mass_center")->view()[0];
    auto cube_inertia = cube_mesh.meta().find<Matrix3x3>("inertia")->view()[0];
    auto cube_volume  = cube_mesh.meta().find<Float>(builtin::volume)->view()[0];

    auto [cube_geo, cube_rest] = cube_object->geometries().create(cube_mesh);

    // --- Body B: proxy with matching mass properties ---
    auto proxy_object = scene.objects().create("proxy");
    auto proxy_mesh   = abd.create_proxy(kappa, cube_mass, cube_com, cube_inertia, cube_volume);

    view(proxy_mesh.transforms())[0] = t0.matrix();

    auto [proxy_geo, proxy_rest] = proxy_object->geometries().create(proxy_mesh);

    // --- Simulate and compare ---
    world.init(scene);
    REQUIRE(world.is_valid());

    while(world.frame() < 25)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();

        Float cube_y  = cube_geo->geometry().transforms().view()[0](1, 3);
        Float proxy_y = proxy_geo->geometry().transforms().view()[0](1, 3);

        REQUIRE(cube_y == Catch::Approx(proxy_y));
    }
}
