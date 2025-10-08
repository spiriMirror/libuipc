#include <algorithm>
#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/compute_mesh_d_hat.h>
#include <uipc/geometry/utils/label_surface.h>

using namespace uipc;
using namespace uipc::geometry;

void compute_mesh_d_hat_test()
{
    vector<Vector3>  Vs = {Vector3{0.0, 0.0, 0.0},
                           Vector3{1.0, 0.0, 0.0},
                           Vector3{0.0, 1.0, 0.0},
                           Vector3{0.0, 0.0, 1.0}};
    vector<Vector4i> Ts = {Vector4i{0, 1, 2, 3}};

    auto tet = tetmesh(Vs, Ts);
    label_surface(tet);

    auto d_hat = compute_mesh_d_hat(tet);

    REQUIRE(d_hat->view()[0] == 1.0);
}

void compute_mesh_d_hat_test(std::string_view mesh)
{
    SimplicialComplexIO io;
    auto                R = io.read(mesh);
    label_surface(R);

    auto d_hat = compute_mesh_d_hat(R, 1.0);

    REQUIRE(d_hat);
    REQUIRE(d_hat->view()[0] <= 1.0);
}

TEST_CASE("compute_mesh_d_hat", "[d_hat]")
{
    compute_mesh_d_hat_test();

    auto tetmesh_path  = AssetDir::tetmesh_path();
    auto trimesh_path  = AssetDir::trimesh_path();
    auto linemesh_path = AssetDir::linemesh_path();

    compute_mesh_d_hat_test(fmt::format("{}bunny0.msh", tetmesh_path));
    compute_mesh_d_hat_test(fmt::format("{}cube.msh", tetmesh_path));
    compute_mesh_d_hat_test(fmt::format("{}ball.msh", tetmesh_path));
    compute_mesh_d_hat_test(fmt::format("{}link.msh", tetmesh_path));

    compute_mesh_d_hat_test(fmt::format("{}cube.obj", trimesh_path));
    compute_mesh_d_hat_test(fmt::format("{}circle.obj", linemesh_path));
}
