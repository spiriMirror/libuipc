#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/compute_vertex_volume.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <iostream>

static constexpr bool DebugPrint = false;

void affine_body_quantity_test(std::string_view msh)
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace Catch;

    SimplicialComplexIO io;

    auto tet = io.read(fmt::format("{}{}.msh", AssetDir::tetmesh_path(), msh));

    Float     m[3];
    Vector3   m_x_bar[3];
    Matrix3x3 m_x_bar_x_bar[3];

    Float rho = 1000.0;
    compute_vertex_volume(tet);

    affine_body::compute_dyadic_mass(tet, rho, m[0], m_x_bar[0], m_x_bar_x_bar[0]);

    label_surface(tet);
    label_triangle_orient(tet);
    auto surf_tet = extract_surface(tet);
    auto tri_tet  = flip_inward_triangles(surf_tet);
    affine_body::compute_dyadic_mass(tri_tet, rho, m[1], m_x_bar[1], m_x_bar_x_bar[1]);
    affine_body::compute_dyadic_mass(surf_tet, rho, m[2], m_x_bar[2], m_x_bar_x_bar[2]);


    Vector3  force_density = rho * Vector3{0, -9.8, 0};
    Vector12 body_force[3];
    body_force[0] = affine_body::compute_body_force(tet, force_density);
    body_force[1] = affine_body::compute_body_force(tri_tet, force_density);
    body_force[2] = affine_body::compute_body_force(surf_tet, force_density);

    if constexpr(DebugPrint)
    {
        for(int i = 0; i < 2; i++)
        {
            std::cout << "M[" << i << "]:" << std::endl;
            std::cout << m[i] << std::endl;
        }

        for(int i = 0; i < 2; i++)
        {
            std::cout << "MX[" << i << "]:" << std::endl;
            std::cout << m_x_bar[i].transpose() << std::endl;
        }

        for(int i = 0; i < 2; i++)
        {
            std::cout << "MXX[" << i << "]:" << std::endl;
            std::cout << m_x_bar_x_bar[i] << std::endl;
        }

        for(int i = 0; i < 2; i++)
        {
            std::cout << "BF[" << i << "]:" << std::endl;
            std::cout << body_force[i].transpose() << std::endl;
        }
    }


    // Consistency Check
    REQUIRE(m[0] == Approx(m[1]));
    REQUIRE(m_x_bar[0].isApprox(m_x_bar[1], 1e-6));
    REQUIRE(m_x_bar_x_bar[0].isApprox(m_x_bar_x_bar[1], 1e-6));
    REQUIRE(body_force[0].isApprox(body_force[1], 1e-6));

    // Functionality Check: We allow `orient` indicator
    REQUIRE(m[0] == Approx(m[2]));
    REQUIRE(m_x_bar[0].isApprox(m_x_bar[2], 1e-6));
    REQUIRE(m_x_bar_x_bar[0].isApprox(m_x_bar_x_bar[2], 1e-6));
    REQUIRE(body_force[0].isApprox(body_force[2], 1e-6));
}


TEST_CASE("affine_body_quantity", "[abd]")
{
    affine_body_quantity_test("tet");
    affine_body_quantity_test("cube");
    affine_body_quantity_test("ball");
    affine_body_quantity_test("link");
    affine_body_quantity_test("bunny0");
}
