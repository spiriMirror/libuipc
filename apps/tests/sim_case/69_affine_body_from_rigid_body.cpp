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

    auto config                 = test::Scene::default_config();
    config["gravity"]           = Vector3{0, -9.8, 0};
    config["contact"]["enable"] = false;
    test::Scene::dump_config(config, output_path);

    Scene scene{config};
    {
        AffineBodyConstitution abd;

        auto object = scene.objects().create("empty_abd");

        // Unit cube rigid body properties:
        //   mass = 1000 kg (rho=1000, a=1)
        //   center of mass = (0.5, 0.5, 0.5)
        //   I_cm = (m*a^2/6) * I_3
        Float     mass = 1000.0;
        Vector3   com{0.5, 0.5, 0.5};
        Matrix3x3 I_cm = (mass / 6.0) * Matrix3x3::Identity();
        Float     volume = 1.0;

        Matrix12x12 M = affine_body::affine_body_from_rigid_body(mass, com, I_cm);

        // Empty mesh: 0 vertices, no collision geometry
        auto mesh = pointcloud(span<const Vector3>{});
        mesh.instances().resize(1);

        abd.apply_to(mesh, 100.0_MPa, M, volume);

        auto trans_view    = view(mesh.transforms());
        auto is_fixed      = mesh.instances().find<IndexT>(builtin::is_fixed);
        auto is_fixed_view = view(*is_fixed);

        Transform t     = Transform::Identity();
        t.translation() = Vector3::UnitY() * 3.0;
        trans_view[0]   = t.matrix();
        is_fixed_view[0] = 0;

        object->geometries().create(mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    while(world.frame() < 20)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
    }
}
