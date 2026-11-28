#include <app/app.h>
#include <app/asset_dir.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/uipc.h>

TEST_CASE("compute_volume", "[volume]")
{
    using namespace uipc;
    using namespace uipc::geometry;

    SimplicialComplexIO io;

    auto  cube = io.read(fmt::format("{}cube.obj", AssetDir::trimesh_path()));
    Float cube_volume = compute_mesh_volume(cube);

    // 1) Correctness check:
    // The volume of a cube with side length 1.0 should be 1.0
    REQUIRE(Catch::Approx(cube_volume).epsilon(1e-6) == 1.0);


    auto tet_ball = io.read(fmt::format("{}ball.msh", AssetDir::tetmesh_path()));

    Float tet_ball_volume = compute_mesh_volume(tet_ball);

    label_surface(tet_ball);
    label_triangle_orient(tet_ball);
    auto surf_ball = extract_surface(tet_ball);
    auto tri_ball  = flip_inward_triangles(surf_ball);

    Float tri_ball_volume = compute_mesh_volume(tri_ball);

    // 2) Consistency check:
    // Tetrahedralized ball volume should be the same as the surface mesh volume
    REQUIRE(Catch::Approx(tet_ball_volume).epsilon(1e-6) == tri_ball_volume);


    Float surf_ball_volume = compute_mesh_volume(surf_ball);

    // 3) Functionality check:
    // Support triangle surface with orient.
    // With the correct orientation, the volume should be the same as the tetrahedralized ball
    REQUIRE(Catch::Approx(surf_ball_volume).epsilon(1e-6) == tet_ball_volume);
}

TEST_CASE("trimesh closure requires exactly two triangles per edge", "[geometry][volume]")
{
    using namespace uipc;
    using namespace uipc::geometry;

    SECTION("closed cube and cube without its top")
    {
        const vector<Vector3> vertices = {
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
        vector<Vector3i> faces = {{0, 2, 1},
                                  {0, 3, 2},
                                  {0, 1, 5},
                                  {0, 5, 4},
                                  {1, 2, 6},
                                  {1, 6, 5},
                                  {2, 3, 7},
                                  {2, 7, 6},
                                  {3, 0, 4},
                                  {3, 4, 7},
                                  {4, 5, 6},
                                  {4, 6, 7}};
        auto             cube  = trimesh(vertices, faces);
        CHECK(is_trimesh_closed(cube));
        CHECK(compute_mesh_volume(cube) == Catch::Approx(1.0));

        faces.resize(10);  // The four open edges join only the highest-numbered vertices.
        auto open = trimesh(vertices, faces);
        CHECK_FALSE(is_trimesh_closed(open));
        CHECK_THROWS_AS(compute_mesh_volume(open), Exception);
        CHECK_THROWS_AS(constitution::AffineBodyConstitution{}.apply_to(open, 100.0, 1.0),
                        Exception);
    }

    SECTION("two tetrahedral surfaces sharing only edge (0,1)")
    {
        const vector<Vector3> vertices = {
            {0, 0, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};
        const vector<Vector3i> faces = {
            {2, 3, 1}, {0, 1, 3}, {0, 2, 1}, {0, 3, 2}, {4, 5, 1}, {0, 1, 5}, {0, 4, 1}, {0, 5, 4}};
        CHECK_FALSE(is_trimesh_closed(trimesh(vertices, faces)));
    }
}
