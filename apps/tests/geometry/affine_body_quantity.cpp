#include <app/test_common.h>
#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/geometry/utils/compute_vertex_volume.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <iostream>

TEST_CASE("affine_body_quantity", "[abd]")
{
    using namespace uipc;
    using namespace uipc::geometry;
    using namespace Catch;

    SimplicialComplexIO io;

    auto tet = io.read(fmt::format("{}ball.msh", AssetDir::tetmesh_path()));

    Float     m[3];
    Vector3   m_x_bar[3];
    Matrix3x3 m_x_bar_x_bar[3];

    compute_vertex_volume(tet);
    tet.meta().create<Float>(builtin::mass_density, 1000.0);

    affine_body::compute_dyadic_mass(tet, m[0], m_x_bar[0], m_x_bar_x_bar[0]);

    label_surface(tet);
    label_triangle_orient(tet);
    auto surf_tet = extract_surface(tet);
    auto tri_tet  = flip_inward_triangles(surf_tet);

    affine_body::compute_dyadic_mass(tri_tet, m[1], m_x_bar[1], m_x_bar_x_bar[1]);

    affine_body::compute_dyadic_mass(surf_tet, m[2], m_x_bar[2], m_x_bar_x_bar[2]);


    for(int i = 0; i < 2; i++)
    {
        std::cout << "M[" << i << "]:" << std::endl;
        std::cout << m[i] << std::endl;
    }

    for(int i = 0; i < 2; i++)
    {
        std::cout << "MX[" << i << "]:" << std::endl;
        std::cout << m_x_bar[i] << std::endl;
    }

    for(int i = 0; i < 2; i++)
    {
        std::cout << "MXX[" << i << "]:" << std::endl;
        std::cout << m_x_bar_x_bar[i] << std::endl;
    }
}
