#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/range.h>
#include <Eigen/Dense>
#include <uipc/geometry/utils/is_trimesh_closed.h>

namespace uipc::geometry::affine_body
{
// ref: libuipc/scripts/symbol_calculation/affine_body_quantity.ipynb

static void compute_tetmesh_dyadic_mass(const SimplicialComplex& sc,
                                        Float                    rho,
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3&               m_x_bar_x_bar)
{
    auto pos_view      = sc.positions().view();
    auto tet_view      = sc.tetrahedra().topo().view();

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    // Integrate the mass density over the volume of the tetrahedra

    for(auto&& [I, tet] : enumerate(tet_view))
    {
        const auto& p0 = pos_view[tet[0]];
        const auto& p1 = pos_view[tet[1]];
        const auto& p2 = pos_view[tet[2]];
        const auto& p3 = pos_view[tet[3]];

        Vector3 r0 = p0;
        Vector3 e1 = p1 - p0;
        Vector3 e2 = p2 - p0;
        Vector3 e3 = p3 - p0;

        Float D = e1.dot(e2.cross(e3));

        m += rho * D / 6.0;

        {
            auto Q = [D](IndexT         i,
                         Float          rho,
                         const Vector3& r0,
                         const Vector3& e1,
                         const Vector3& e2,
                         const Vector3& e3)
            {
                Float V = 0.0;

                V += r0(i) / 6;
                V += e1(i) / 24;
                V += e2(i) / 24;
                V += e3(i) / 24;

                return rho * D * V;
            };


            for(IndexT i = 0; i < 3; i++)
            {
                m_x_bar(i) += Q(i, rho, r0, e1, e2, e3);
            }
        }

        {
            auto Q = [D](IndexT         a,
                         IndexT         b,
                         Float          rho,
                         const Vector3& r0,
                         const Vector3& e1,
                         const Vector3& e2,
                         const Vector3& e3)
            {
                Float V = 0.0;

                V += e1(a) * e1(b) / 60;
                V += e1(a) * e2(b) / 120;
                V += e1(a) * e3(b) / 120;
                V += e1(a) * r0(b) / 24;

                V += e1(b) * e2(a) / 120;
                V += e1(b) * e3(a) / 120;
                V += e1(b) * r0(a) / 24;
                V += e2(a) * e2(b) / 60;

                V += e2(a) * e3(b) / 120;
                V += e2(a) * r0(b) / 24;
                V += e2(b) * e3(a) / 120;
                V += e2(b) * r0(a) / 24;

                V += e3(a) * e3(b) / 60;
                V += e3(a) * r0(b) / 24;
                V += e3(b) * r0(a) / 24;
                V += r0(a) * r0(b) / 6;

                return rho * D * V;
            };

            for(IndexT a = 0; a < 3; a++)
            {
                for(IndexT b = 0; b < 3; b++)
                {
                    m_x_bar_x_bar(a, b) += Q(a, b, rho, r0, e1, e2, e3);
                }
            }
        }
    }
}

static void compute_trimesh_dyadic_mass(const SimplicialComplex& sc,
                                        Float                    rho,
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3&               m_x_bar_x_bar)
{
    auto pos_view    = sc.positions().view();
    auto tri_view    = sc.triangles().topo().view();
    auto orient      = sc.triangles().find<IndexT>(builtin::orient);
    auto orient_view = orient ? orient->view() : span<const IndexT>{};

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    // Using Divergence theorem to compute the dyadic mass
    // by integrating on the surface of the trimesh

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 e1 = p1 - p0;
        Vector3 e2 = p2 - p0;

        Vector3 N = e1.cross(e2);
        if(orient && orient_view[i] < 0)
            N = -N;

        //tex:
        //$$
        // m_b = \int_{\Omega} \rho dV = \int_{\partial \Omega} \frac{\rho}{3} \mathbf{x} \cdot d\mathbf{S}
        //$$

        m += rho * p0.dot(N) / 6.0;

        //tex:
        //$$
        // (mx)_b = \int_{\Omega} \rho x dV = \int_{\partial \Omega}
        // \begin{bmatrix}
        // \frac{\rho}{2}{x}^2 \\
        // 0 \\
        // 0
        // \end{bmatrix}
        //
        // \cdot d\mathbf{S}
        //$$


        {
            auto Q = [](IndexT         a,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& r0,
                        const Vector3& e1,
                        const Vector3& e2)
            {
                Float V = 0.0;

                V += e1(a) * e1(a) / 12;
                V += e1(a) * e2(a) / 12;
                V += e1(a) * r0(a) / 3;

                V += e2(a) * e2(a) / 12;
                V += e2(a) * r0(a) / 3;
                V += r0(a) * r0(a) / 2;

                return rho / 2 * N(a) * V;
            };

            for(IndexT a = 0; a < 3; a++)
            {
                m_x_bar(a) += Q(a, rho, N, p0, e1, e2);
            }
        }

        //tex:
        //$$
        // (mxx)_b = \int_{\Omega} \rho x^2 dV = \int_{\partial \Omega}
        // \begin{bmatrix}
        // \frac{\rho}{3} x^3 \\
        // 0 \\
        // 0 \\
        // \end{bmatrix}
        // \cdot
        // d\mathbf{S}
        //$$


        {
            auto Q = [](IndexT         a,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& r0,
                        const Vector3& e1,
                        const Vector3& e2)
            {
                Float V = 0.0;

                Float e1a_2 = e1(a) * e1(a);
                Float e2a_2 = e2(a) * e2(a);
                Float r0a_2 = r0(a) * r0(a);

                Float e1a_3 = e1a_2 * e1(a);
                Float e2a_3 = e2a_2 * e2(a);
                Float r0a_3 = r0a_2 * r0(a);

                V += e1a_3 / 20;
                V += e1a_2 * e2(a) / 20;
                V += e1a_2 * r0(a) / 4;

                V += e1(a) * e2a_2 / 20;
                V += e1(a) * e2(a) * r0(a) / 4;
                V += e1(a) * r0a_2 / 2;

                V += e2a_3 / 20;
                V += e2a_2 * r0(a) / 4;
                V += e2(a) * r0a_2 / 2;

                V += r0a_3 / 2;

                return rho / 3 * N(a) * V;
            };

            m_x_bar_x_bar(0, 0) += Q(0, rho, N, p0, e1, e2);
            m_x_bar_x_bar(1, 1) += Q(1, rho, N, p0, e1, e2);
            m_x_bar_x_bar(2, 2) += Q(2, rho, N, p0, e1, e2);
        }

        //tex:
        //$$
        // (mxy)_b = \int_{\Omega} \rho xy dV = \int_{\partial \Omega}
        // \begin{bmatrix}
        // \frac{1}{2} \rho x^2 y \\ 
        // 0 \\
        // 0
        // \end{bmatrix}
        // \cdot
        // d\mathbf{S}
        //$$

        {
            auto Q = [](IndexT         a,
                        IndexT         b,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& r0,
                        const Vector3& e1,
                        const Vector3& e2)
            {
                Float V = 0.0;

                Float e1a_2 = e1(a) * e1(a);
                Float e2a_2 = e2(a) * e2(a);
                Float r0a_2 = r0(a) * r0(a);

                V += e1a_2 * e1(b) / 20;
                V += e1a_2 * e2(b) / 60;
                V += e1a_2 * r0(b) / 12;
                V += e1(a) * e1(b) * e2(a) / 30;

                V += e1(a) * e1(b) * r0(a) / 6;
                V += e1(a) * e2(a) * e2(b) / 30;
                V += e1(a) * e2(a) * r0(b) / 12;
                V += e1(a) * e2(b) * r0(a) / 12;

                V += e1(a) * r0(a) * r0(b) / 3;
                V += e1(b) * e2a_2 / 60;
                V += e1(b) * e2(a) * r0(a) / 12;
                V += e1(b) * r0a_2 / 6;

                V += e2a_2 * e2(b) / 20;
                V += e2a_2 * r0(b) / 12;
                V += e2(a) * e2(b) * r0(a) / 6;
                V += e2(a) * r0(a) * r0(b) / 3;

                V += e2(b) * r0a_2 / 6;
                V += r0a_2 * r0(b) / 2;

                return rho / 2 * N(a) * V;
            };

            m_x_bar_x_bar(0, 1) += Q(0, 1, rho, N, p0, e1, e2);
            m_x_bar_x_bar(0, 2) += Q(0, 2, rho, N, p0, e1, e2);
            m_x_bar_x_bar(1, 2) += Q(1, 2, rho, N, p0, e1, e2);

            // symmetric

            m_x_bar_x_bar(1, 0) = m_x_bar_x_bar(0, 1);
            m_x_bar_x_bar(2, 0) = m_x_bar_x_bar(0, 2);
            m_x_bar_x_bar(2, 1) = m_x_bar_x_bar(1, 2);
        }
    }
}

UIPC_GEOMETRY_API void compute_dyadic_mass(const SimplicialComplex& sc,
                                           Float                    rho,
                                           Float&                   m,
                                           Vector3&                 m_x_bar,
                                           Matrix3x3& m_x_bar_x_bar)
{
    if(sc.dim() == 3)
    {
        compute_tetmesh_dyadic_mass(sc, rho, m, m_x_bar, m_x_bar_x_bar);
    }
    else if(sc.dim() == 2)
    {
        // UIPC_ASSERT(is_trimesh_closed(sc), "Only closed trimesh is supported.");
        compute_trimesh_dyadic_mass(sc, rho, m, m_x_bar, m_x_bar_x_bar);
    }
    else
    {
        UIPC_ASSERT(false, "Unsupported dimension");
    }
}
}  // namespace uipc::geometry::affine_body
