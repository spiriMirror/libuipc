#include <algorithm>
#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

using namespace uipc;
using namespace uipc::geometry;

TEST_CASE("optimal_transform", "[optimal_transform]")
{
    vector<Vector3>  Vs = {Vector3{0.0, 0.0, 0.0},
                           Vector3{1.0, 0.0, 0.0},
                           Vector3{0.0, 1.0, 0.0},
                           Vector3{0.0, 0.0, 1.0}};
    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto S = tetmesh(Vs, Ts);

    Transform t = Transform::Identity();
    t.scale(2.0);
    t.rotate(Eigen::AngleAxisd(0.5, Vector3::UnitX()));
    t.translate(Vector3{1.0, 2.0, 3.0});
    auto res_mat = t.matrix();

    std::ranges::transform(
        Vs, Vs.begin(), [&t](const Vector3& v) { return t * v; });

    auto D = tetmesh(Vs, Ts);

    auto M = optimal_transform(S, D);

    REQUIRE(M.isApprox(res_mat));
}

void optimal_transform_mesh_test(std::string_view mesh)
{
    Transform t  = Transform::Identity();
    Vector4   SQ = Vector4::Random();
    t.scale(SQ.segment<3>(0));
    t.rotate(Eigen::AngleAxisd(SQ.w(), Vector3::Random().normalized()));
    t.translate(Vector3::Random());
    auto res_mat = t.matrix();


    SimplicialComplexIO ioS;
    SimplicialComplexIO ioD{t};

    auto S = ioS.read(mesh);
    auto D = ioD.read(mesh);

    auto M = optimal_transform(S, D);

    REQUIRE(M.isApprox(res_mat));
}

TEST_CASE("optimal_transform_mesh", "[optimal_transform]")
{
    auto tetmesh_path = AssetDir::tetmesh_path();
    optimal_transform_mesh_test(fmt::format("{}/bunny0.msh", tetmesh_path));
    optimal_transform_mesh_test(fmt::format("{}/cube.msh", tetmesh_path));
    optimal_transform_mesh_test(fmt::format("{}/ball.msh", tetmesh_path));
    optimal_transform_mesh_test(fmt::format("{}/link.msh", tetmesh_path));
}
