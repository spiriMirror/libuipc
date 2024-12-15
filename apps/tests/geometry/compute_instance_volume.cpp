#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>

TEST_CASE("compute_instance_volume", "[volume]")
{
    using namespace uipc;
    using namespace uipc::geometry;

    SimplicialComplexIO io;

    auto compute_volume = [](SimplicialComplex& mesh) -> Float
    {
        auto volume_attr = compute_instance_volume(mesh);
        auto volume_view = volume_attr->view();
        return volume_view[0];
    };

    auto  cube = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    Float cube_volume = compute_volume(cube);

    // 1) Correctness check:
    // The volume of a cube with side length 1.0 should be 1.0
    REQUIRE(Catch::Approx(cube_volume).epsilon(1e-6) == 1.0);


    auto tet_ball = io.read(fmt::format("{}ball.msh", AssetDir::tetmesh_path()));

    Float tet_ball_volume = compute_volume(tet_ball);

    label_surface(tet_ball);
    label_triangle_orient(tet_ball);
    auto surf_ball = extract_surface(tet_ball);
    auto tri_ball  = flip_inward_triangles(surf_ball);

    Float tri_ball_volume = compute_volume(tri_ball);

    // 2) Consistency check:
    // Tetrahedralized ball volume should be the same as the surface mesh volume
    REQUIRE(Catch::Approx(tet_ball_volume).epsilon(1e-6) == tri_ball_volume);


    Float surf_ball_volume = compute_volume(surf_ball);

    // 3) Functionality check:
    // Support triangle surface with orient.
    // With the correct orientation, the volume should be the same as the tetrahedralized ball
    REQUIRE(Catch::Approx(surf_ball_volume).epsilon(1e-6) == tet_ball_volume);
}
