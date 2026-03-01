#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/range.h>
#include <Eigen/Dense>
#include <uipc/geometry/utils/is_trimesh_closed.h>
#include <numbers>

namespace uipc::geometry::affine_body
{
// ref: libuipc/scripts/symbol_calculation/affine_body_quantity.ipynb

static void compute_tetmesh_dyadic_mass(const SimplicialComplex& sc,
                                        Float                    rho,
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3&               m_x_bar_x_bar)
{
    auto pos_view = sc.positions().view();
    auto tet_view = sc.tetrahedra().topo().view();

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
            auto Q = [D](IndexT         a,
                         Float          rho,
                         const Vector3& p0,
                         const Vector3& p1,
                         const Vector3& p2,
                         const Vector3& p3)
            {
                Float V = 0.0;

                V += p0(a) / 24;
                V += p1(a) / 24;
                V += p2(a) / 24;
                V += p3(a) / 24;

                return rho * D * V;
            };

            for(IndexT i = 0; i < 3; i++)
            {
                m_x_bar(i) += Q(i, rho, p0, p1, p2, p3);
            }
        }

        {
            auto Q = [D](IndexT         a,
                         IndexT         b,
                         Float          rho,
                         const Vector3& p0,
                         const Vector3& p1,
                         const Vector3& p2,
                         const Vector3& p3)
            {
                Float V = 0.0;

                V += p0(a) * p0(b) / 60;
                V += p0(a) * p1(b) / 120;
                V += p0(a) * p2(b) / 120;
                V += p0(a) * p3(b) / 120;

                V += p0(b) * p1(a) / 120;
                V += p0(b) * p2(a) / 120;
                V += p0(b) * p3(a) / 120;
                V += p1(a) * p1(b) / 60;

                V += p1(a) * p2(b) / 120;
                V += p1(a) * p3(b) / 120;
                V += p1(b) * p2(a) / 120;
                V += p1(b) * p3(a) / 120;

                V += p2(a) * p2(b) / 60;
                V += p2(a) * p3(b) / 120;
                V += p2(b) * p3(a) / 120;
                V += p3(a) * p3(b) / 60;

                return rho * D * V;
            };

            for(IndexT a = 0; a < 3; a++)
                for(IndexT b = 0; b < 3; b++)
                    m_x_bar_x_bar(a, b) += Q(a, b, rho, p0, p1, p2, p3);
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

        m += rho * p0.dot(N) / 6.0;


        {

            auto Q = [](IndexT         a,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& p0,
                        const Vector3& p1,
                        const Vector3& p2)
            {
                Float V = 0.0;

                V += p0(a) * p0(a) / 12;
                V += p0(a) * p1(a) / 12;
                V += p0(a) * p2(a) / 12;

                V += p1(a) * p1(a) / 12;
                V += p1(a) * p2(a) / 12;
                V += p2(a) * p2(a) / 12;

                return rho / 2 * N(a) * V;
            };

            for(IndexT a = 0; a < 3; a++)
            {
                m_x_bar(a) += Q(a, rho, N, p0, p1, p2);
            }
        }

        {
            auto Q = [](IndexT         a,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& p0,
                        const Vector3& p1,
                        const Vector3& p2)
            {
                Float V = 0.0;

                Float p0a_2 = p0(a) * p0(a);
                Float p1a_2 = p1(a) * p1(a);
                Float p2a_2 = p2(a) * p2(a);

                Float p0a_3 = p0a_2 * p0(a);
                Float p1a_3 = p1a_2 * p1(a);
                Float p2a_3 = p2a_2 * p2(a);

                V += p0a_3 / 20;
                V += p0a_2 * p1(a) / 20;
                V += p0a_2 * p2(a) / 20;

                V += p0(a) * p1a_2 / 20;
                V += p0(a) * p1(a) * p2(a) / 20;
                V += p0(a) * p2a_2 / 20;

                V += p1a_3 / 20;
                V += p1a_2 * p2(a) / 20;
                V += p1(a) * p2a_2 / 20;

                V += p2a_3 / 20;

                return rho / 3 * N(a) * V;
            };

            for(IndexT i = 0; i < 3; i++)  // diagonal
                m_x_bar_x_bar(i, i) += Q(i, rho, N, p0, p1, p2);
        }

        {
            auto Q = [](IndexT         a,
                        IndexT         b,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& p0,
                        const Vector3& p1,
                        const Vector3& p2)
            {
                Float V = 0.0;

                Float p0a_2 = p0(a) * p0(a);
                Float p1a_2 = p1(a) * p1(a);
                Float p2a_2 = p2(a) * p2(a);

                V += p0a_2 * p0(b) / 20;
                V += p0a_2 * p1(b) / 60;
                V += p0a_2 * p2(b) / 60;
                V += p0(a) * p0(b) * p1(a) / 30;

                V += p0(a) * p0(b) * p2(a) / 30;
                V += p0(a) * p1(a) * p1(b) / 30;
                V += p0(a) * p1(a) * p2(b) / 60;
                V += p0(a) * p1(b) * p2(a) / 60;

                V += p0(a) * p2(a) * p2(b) / 30;
                V += p0(b) * p1a_2 / 60;
                V += p0(b) * p1(a) * p2(a) / 60;
                V += p0(b) * p2a_2 / 60;

                V += p1a_2 * p1(b) / 20;
                V += p1a_2 * p2(b) / 60;
                V += p1(a) * p1(b) * p2(a) / 30;
                V += p1(a) * p2(a) * p2(b) / 30;

                V += p1(b) * p2a_2 / 60;
                V += p2a_2 * p2(b) / 20;

                return rho / 2 * N(a) * V;
            };

            m_x_bar_x_bar(0, 1) += Q(0, 1, rho, N, p0, p1, p2);
            m_x_bar_x_bar(0, 2) += Q(0, 2, rho, N, p0, p1, p2);
            m_x_bar_x_bar(1, 2) += Q(1, 2, rho, N, p0, p1, p2);
        }

        // symmetric
        m_x_bar_x_bar(1, 0) = m_x_bar_x_bar(0, 1);
        m_x_bar_x_bar(2, 0) = m_x_bar_x_bar(0, 2);
        m_x_bar_x_bar(2, 1) = m_x_bar_x_bar(1, 2);
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

// ============================================================================
// Codimensional dyadic mass (with thickness)
// ref: libuipc/scripts/symbol_calculation/codim_abd_quantity.ipynb
// ============================================================================

// Shell (codim 2D): direct area integration over triangles
// with slab-thickness correction for full-rank mass matrix.
static void compute_codim_trimesh_dyadic_mass(const SimplicialComplex& sc,
                                              Float                    rho,
                                              Float                    thickness,
                                              Float&                   m,
                                              Vector3&                 m_x_bar,
                                              Matrix3x3& m_x_bar_x_bar)
{
    auto pos_view = sc.positions().view();
    auto tri_view = sc.triangles().topo().view();

    Float r    = thickness;
    Float twor = 2.0 * r;

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 e1     = p1 - p0;
        Vector3 e2     = p2 - p0;
        Vector3 cross  = e1.cross(e2);
        Float   D      = cross.norm();  // |e1 x e2|
        Float   area   = 0.5 * D;
        Vector3 n_hat  = (D > 0.0) ? (cross / D).eval() : Vector3::Zero();

        // ----- mass -----
        // m_i = rho * area * 2r
        m += rho * area * twor;

        // ----- first moment -----
        // mx^a_i = rho * |e1xe2| * 2r * (p0+p1+p2)^a / 6
        //        = rho * D * 2r * (p0+p1+p2) / 6
        {
            auto Q = [&](IndexT a) -> Float
            { return (p0(a) + p1(a) + p2(a)) / 6.0; };

            for(IndexT a = 0; a < 3; a++)
                m_x_bar(a) += rho * D * twor * Q(a);
        }

        // ----- second moment (in-plane) -----
        // mxx^{ab}_i = rho * |e1xe2| * 2r *
        //   (2*sum_j p_j^a p_j^b + sum_{j!=k} p_j^a p_k^b) / 24
        {
            auto Q = [&](IndexT a, IndexT b) -> Float
            {
                Float V = 0.0;
                // diagonal (j==k) terms — coefficient 2/24 = 1/12
                V += p0(a) * p0(b);
                V += p1(a) * p1(b);
                V += p2(a) * p2(b);
                V *= 2.0;
                // off-diagonal (j!=k) terms — coefficient 1/24
                V += p0(a) * p1(b) + p1(a) * p0(b);
                V += p0(a) * p2(b) + p2(a) * p0(b);
                V += p1(a) * p2(b) + p2(a) * p1(b);
                return V / 24.0;
            };

            for(IndexT a = 0; a < 3; a++)
                for(IndexT b = 0; b < 3; b++)
                    m_x_bar_x_bar(a, b) += rho * D * twor * Q(a, b);
        }

        // ----- second moment: normal (slab) correction -----
        // += rho * area * (2r^3/3) * n_hat (x) n_hat
        {
            Float coeff = rho * area * (2.0 * r * r * r / 3.0);
            m_x_bar_x_bar += coeff * (n_hat * n_hat.transpose());
        }
    }
}

// Rod (codim 1D): segment integration with cylindrical cross-section correction
// for full-rank mass matrix.
static void compute_segment_dyadic_mass(const SimplicialComplex& sc,
                                        Float                    rho,
                                        Float                    thickness,
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3& m_x_bar_x_bar)
{
    auto pos_view  = sc.positions().view();
    auto edge_view = sc.edges().topo().view();

    Float r  = thickness;
    Float pi = std::numbers::pi_v<Float>;
    Float S  = pi * r * r;  // cross-section area pi*r^2

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    for(auto&& [i, E] : enumerate(edge_view))
    {
        const auto& p0 = pos_view[E[0]];
        const auto& p1 = pos_view[E[1]];

        Vector3 e1   = p1 - p0;
        Float   L    = e1.norm();
        Vector3 t_hat = (L > 0.0) ? (e1 / L).eval() : Vector3::Zero();

        // ----- mass -----
        // m_i = rho * L * pi * r^2
        m += rho * L * S;

        // ----- first moment -----
        // mx^a_i = rho * L * S * (p0+p1)^a / 2
        {
            auto Q = [&](IndexT a) -> Float
            { return (p0(a) + p1(a)) / 2.0; };

            for(IndexT a = 0; a < 3; a++)
                m_x_bar(a) += rho * L * S * Q(a);
        }

        // ----- second moment (along-axis) -----
        // mxx^{ab}_i = rho * L * S *
        //   (2*p0^a*p0^b + p0^a*p1^b + p1^a*p0^b + 2*p1^a*p1^b) / 6
        {
            auto Q = [&](IndexT a, IndexT b) -> Float
            {
                Float V = 0.0;
                V += 2.0 * p0(a) * p0(b);
                V += p0(a) * p1(b);
                V += p1(a) * p0(b);
                V += 2.0 * p1(a) * p1(b);
                return V / 6.0;
            };

            for(IndexT a = 0; a < 3; a++)
                for(IndexT b = 0; b < 3; b++)
                    m_x_bar_x_bar(a, b) += rho * L * S * Q(a, b);
        }

        // ----- second moment: cross-section correction -----
        // += rho * L * (pi * r^4 / 4) * (I - t_hat (x) t_hat)
        {
            Float   Ics   = pi * r * r * r * r / 4.0;
            Float   coeff = rho * L * Ics;
            Matrix3x3 P = Matrix3x3::Identity() - t_hat * t_hat.transpose();
            m_x_bar_x_bar += coeff * P;
        }
    }
}

UIPC_GEOMETRY_API void compute_dyadic_mass(const SimplicialComplex& sc,
                                           Float                    rho,
                                           Float                    thickness,
                                           Float&                   m,
                                           Vector3&                 m_x_bar,
                                           Matrix3x3& m_x_bar_x_bar)
{
    if(sc.dim() == 2)
    {
        compute_codim_trimesh_dyadic_mass(
            sc, rho, thickness, m, m_x_bar, m_x_bar_x_bar);
    }
    else if(sc.dim() == 1)
    {
        compute_segment_dyadic_mass(
            sc, rho, thickness, m, m_x_bar, m_x_bar_x_bar);
    }
    else
    {
        UIPC_ASSERT(false,
                    "Codim compute_dyadic_mass only supports dim==2 (shell) and dim==1 (rod), got dim={}.",
                    sc.dim());
    }
}
}  // namespace uipc::geometry::affine_body
