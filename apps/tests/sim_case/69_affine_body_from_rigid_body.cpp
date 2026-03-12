#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/geometry/utils/affine_body/affine_body_from_rigid_body.h>

TEST_CASE("69_affine_body_from_rigid_body", "[abd]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

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

    // Use c=(0,0,0) so the mass matrix is block-diagonal:
    // translation DOFs decouple from affine DOFs,
    // giving a clean free-fall check.
    Float     mass   = 1000.0;
    Vector3   com    = Vector3::Zero();
    Matrix3x3 I_cm   = (mass / 6.0) * Matrix3x3::Identity();
    Float     volume = 1.0;

    Vector3 initial_pos = Vector3::UnitY() * 3.0;

    Scene scene{config};

    AffineBodyConstitution abd;
    auto object = scene.objects().create("free_fall_abd");

    Matrix12x12 M = affine_body::from_rigid_body(mass, com, I_cm);

    auto mesh = pointcloud(span<const Vector3>{});
    mesh.instances().resize(1);

    abd.apply_to(mesh, 100.0_MPa, M, volume);

    auto trans_view    = view(mesh.transforms());
    auto is_fixed_view = view(*mesh.instances().find<IndexT>(builtin::is_fixed));

    Transform t     = Transform::Identity();
    t.translation() = initial_pos;
    trans_view[0]   = t.matrix();
    is_fixed_view[0] = 0;

    auto [geo_slot, rest_geo_slot] = object->geometries().create(mesh);

    world.init(scene);
    REQUIRE(world.is_valid());

    world.advance();
    REQUIRE(world.is_valid());
    world.retrieve();

    // After the first frame, the translation should be initial_pos + g * dt^2.
    // With c=0 the mass matrix is block-diagonal, so translation is decoupled.
    auto&     sc  = geo_slot->geometry();
    Matrix4x4 T   = sc.transforms().view()[0];
    Vector3   pos = T.block<3, 1>(0, 3);
    Vector3   expected = initial_pos + gravity * dt * dt;
    REQUIRE(pos.isApprox(expected));

    while(world.frame() < 20)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
    }
}
