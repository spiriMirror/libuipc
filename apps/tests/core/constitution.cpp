#include <catch2/catch_all.hpp>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_shell.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>

TEST_CASE("abd", "[constitution]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::constitution;

    Scene                  scene;
    AffineBodyConstitution abd;
    scene.constitution_tabular().insert(abd);

    geometry::SimplicialComplexIO io;
    auto mesh0 = io.read_msh(fmt::format("{}cube.msh", AssetDir::tetmesh_path()));
    mesh0.instances().resize(5);

    abd.apply_to(mesh0, 1e8);

    auto kappa = mesh0.instances().find<Float>("kappa");

    REQUIRE(kappa->size() == mesh0.instances().size());
    REQUIRE(std::ranges::all_of(kappa->view(), [](auto v) { return v == 1e8; }));

    // this name is important for the AffineBodyConstitution
    REQUIRE(kappa->name() == "kappa");
}

TEST_CASE("ABD reapplication and material-frame motors", "[constitution][abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    SimplicialComplexIO io;
    auto mesh = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    AffineBodyShell{}.apply_to(mesh, 1e6, 1.0, 0.1);
    AffineBodyConstitution{}.apply_to(mesh, 1e6, 1.0);
    REQUIRE(mesh.meta().find<IndexT>(builtin::is_codim)->view()[0] == 0);
    const Vector12 load = affine_body::compute_body_force(mesh, Vector3::UnitX());
    REQUIRE(load[0] == Catch::Approx(1.0));

    Transform pose = Transform::Identity();
    pose.translate(Vector3{0, 2, 0});
    pose.rotate(Eigen::AngleAxisd{std::numbers::pi / 2, Vector3::UnitZ()});
    view(mesh.transforms())[0] = pose.matrix();
    SECTION("rotation about a local axis preserves translation")
    {
        RotatingMotor{}.apply_to(mesh, 100, Vector3::UnitX(), 1);
        view(*mesh.instances().find<IndexT>(builtin::is_constrained))[0] = 1;
        RotatingMotor::animate(mesh, 0.25);
        pose.rotate(Eigen::AngleAxisd{0.25, Vector3::UnitX()});
    }
    SECTION("linear motion follows the rotated material axis")
    {
        LinearMotor{}.apply_to(mesh, 100, Vector3::UnitX(), 1);
        view(*mesh.instances().find<IndexT>(builtin::is_constrained))[0] = 1;
        LinearMotor::animate(mesh, 0.25);
        pose.translate(0.25 * Vector3::UnitX());
    }
    const auto target = mesh.instances().find<Matrix4x4>(builtin::aim_transform);
    REQUIRE(target->view()[0].isApprox(pose.matrix()));
}
