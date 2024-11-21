#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/range.h>
#include <Eigen/Dense>
#include <uipc/geometry/utils/is_trimesh_closed.h>
#include <triangle_quatrature.h>
#include <tetrahedron_quatrature.h>

namespace uipc::geometry::affine_body
{
static void compute_tetmesh_dyadic_mass(const SimplicialComplex& sc,
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3&               m_x_bar_x_bar)
{
    auto pos_view      = sc.positions().view();
    auto tet_view      = sc.tetrahedra().topo().view();
    auto vertex_volume = sc.vertices().find<Float>(builtin::volume);

    UIPC_ASSERT(vertex_volume, "Attribute `volume` not found on tetmesh vertices, why can it happen?");
    auto vertex_volume_view = vertex_volume->view();

    auto meta_mass_density = sc.meta().find<Float>(builtin::mass_density);
    auto meta_mass_density_view =
        meta_mass_density ? meta_mass_density->view() : span<const Float>{};

    auto vertex_mass_density = sc.vertices().find<Float>(builtin::mass_density);
    auto vertex_mass_density_view =
        vertex_mass_density ? vertex_mass_density->view() : span<const Float>{};

    UIPC_ASSERT(meta_mass_density || vertex_mass_density,
                "Attribute `mass_density` not found on tetmesh, please provide it on vertices or meta");

    auto rho = meta_mass_density_view[0];

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    //for(auto&& i : range(pos_view.size()))
    //{
    //    auto rho = vertex_mass_density_view.empty() ? meta_mass_density_view[0] :
    //                                                  vertex_mass_density_view[i];

    //    auto mi = rho * vertex_volume_view[i];

    //    m += mi;
    //    m_x_bar += mi * pos_view[i];
    //    m_x_bar_x_bar += mi * pos_view[i] * pos_view[i].transpose();
    //}

    using TQ = TetrahedronQuatrature;

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
                V += r0(i) * TQ::I_1_dudvdw;
                V += e1(i) * TQ::I_u_dudvdw;
                V += e2(i) * TQ::I_v_dudvdw;
                V += e3(i) * TQ::I_w_dudvdw;
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

                V += e1(a) * e1(b) * TQ::I_uu_dudvdw;
                V += e2(a) * e2(b) * TQ::I_vv_dudvdw;
                V += e3(a) * e3(b) * TQ::I_ww_dudvdw;

                V += (e1(a) * e2(b) + e1(b) * e2(a)) * TQ::I_uv_dudvdw;
                V += (e1(a) * e3(b) + e1(b) * e3(a)) * TQ::I_uw_dudvdw;
                V += (e2(a) * e3(b) + e2(b) * e3(a)) * TQ::I_vw_dudvdw;

                V += (e1(a) * r0(b) + e1(b) * r0(a)) * TQ::I_u_dudvdw;
                V += (e2(a) * r0(b) + e2(b) * r0(a)) * TQ::I_v_dudvdw;
                V += (e3(a) * r0(b) + e3(b) * r0(a)) * TQ::I_w_dudvdw;

                V += r0(a) * r0(b) * TQ::I_1_dudvdw;

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
                                        Float&                   m,
                                        Vector3&                 m_x_bar,
                                        Matrix3x3&               m_x_bar_x_bar)
{
    auto pos_view    = sc.positions().view();
    auto tri_view    = sc.triangles().topo().view();
    auto orient      = sc.triangles().find<IndexT>(builtin::orient);
    auto orient_view = orient ? orient->view() : span<const IndexT>{};

    auto mass_density = sc.meta().find<Float>(builtin::mass_density);

    UIPC_ASSERT(mass_density, "Attribute `density` not found on trimesh, why can it happen?");

    auto vertex_mass_density = sc.vertices().find<Float>(builtin::mass_density);

    if(vertex_mass_density)
    {
        UIPC_WARN_WITH_LOCATION(
            "Attribute `mass_density` found on vertices, but it is ignored, "
            "we can't calculate dyadic mass for trimesh with non-uniform mass density");
    }

    m = 0.0;
    m_x_bar.setZero();
    m_x_bar_x_bar.setZero();

    auto rho = mass_density->view()[0];

    // Using Divergence theorem to compute the dyadic mass
    // by integrating on the surface of the trimesh

    using TQ = TriangleQuatrature;

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
            auto Q = [](IndexT         i,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& r0,
                        const Vector3& e1,
                        const Vector3& e2)
            {
                return rho / 2 * N(i)
                       * (r0(i) * r0(i) * TQ::I_1_dudv         //
                          + e1(i) * e1(i) * TQ::I_uu_dudv      //
                          + e2(i) * e2(i) * TQ::I_vv_dudv      //
                          + r0(i) * e1(i) * 2 * TQ::I_u_dudv   //
                          + r0(i) * e2(i) * 2 * TQ::I_v_dudv   //
                          + e1(i) * e2(i) * 2 * TQ::I_uv_dudv  //
                       );
            };

            m_x_bar.x() += Q(0, rho, N, p0, e1, e2);
            m_x_bar.y() += Q(1, rho, N, p0, e1, e2);
            m_x_bar.z() += Q(2, rho, N, p0, e1, e2);
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
            auto Q = [](IndexT         i,
                        Float          rho,
                        const Vector3& N,
                        const Vector3& r0,
                        const Vector3& e1,
                        const Vector3& e2)
            {
                Float V = 0.0;

                V += std::pow(r0(i), 3) * TQ::I_1_dudv;
                V += std::pow(e1(i), 3) * TQ::I_uuu_dudv;
                V += std::pow(e2(i), 3) * TQ::I_vvv_dudv;

                Float r_2  = r0(i) * r0(i);
                Float e1_2 = e1(i) * e1(i);
                Float e2_2 = e2(i) * e2(i);

                V += r_2 * e1(i) * 3 * TQ::I_u_dudv;
                V += r_2 * e2(i) * 3 * TQ::I_v_dudv;

                V += e1_2 * r0(i) * 3 * TQ::I_uu_dudv;
                V += e2_2 * r0(i) * 3 * TQ::I_vv_dudv;

                V += e1_2 * e2(i) * 3 * TQ::I_uuv_dudv;
                V += e2_2 * e1(i) * 3 * TQ::I_uvv_dudv;

                V += r0(i) * e1(i) * e2(i) * 6 * TQ::I_uv_dudv;

                return rho / 3 * N(i) * V;
            };

            m_x_bar_x_bar(0, 0) += Q(0, rho, N, p0, e1, e2);
            m_x_bar_x_bar(1, 1) += Q(1, rho, N, p0, e1, e2);
            m_x_bar_x_bar(2, 2) += Q(2, rho, N, p0, e1, e2);
        }

        //m_x_bar_x_bar(0, 0) += rho * X.x() * X.x() * X.x() * S.x() / 3.0;
        //m_x_bar_x_bar(1, 1) += rho * X.y() * X.y() * X.y() * S.y() / 3.0;
        //m_x_bar_x_bar(2, 2) += rho * X.z() * X.z() * X.z() * S.z() / 3.0;

        //tex:
        //$$
        // (mxy)_b = \int_{\Omega} \rho xy dV = \int_{\partial \Omega}
        // \begin{bmatrix}
        // 0 \\
        // 0 \\
        // \rho x y \\
        // \end{bmatrix}
        // \cdot
        // d\mathbf{S}
        // \approx \sum_{k} \rho x_k y_k S^z_k
        //$$
        // so for other off-diagonal terms.
        //$$
        // (mxz)_b \approx \sum_{k} \rho x_k z_k S^y_k
        //$$
        //$$
        // (myx)_b \approx \sum_{k} \rho y_k x_k S^z_k
        //$$
        //$$
        // (myz)_b \approx \sum_{k} \rho y_k z_k S^x_k
        //$$
        //$$
        // (mzx)_b \approx \sum_{k} \rho z_k x_k S^y_k
        //$$
        //$$
        // (mzy)_b \approx \sum_{k} \rho z_k y_k S^x_k
        //$$

        //m_x_bar_x_bar(0, 1) += rho * X.x() * X.y() * S.z();
        //m_x_bar_x_bar(0, 2) += rho * X.x() * X.z() * S.y();

        //m_x_bar_x_bar(1, 0) += rho * X.y() * X.x() * S.z();
        //m_x_bar_x_bar(1, 2) += rho * X.y() * X.z() * S.x();

        //m_x_bar_x_bar(2, 0) += rho * X.z() * X.x() * S.y();
        //m_x_bar_x_bar(2, 1) += rho * X.z() * X.y() * S.x();
    }
}

UIPC_GEOMETRY_API void compute_dyadic_mass(const SimplicialComplex& sc,
                                           Float&                   m,
                                           Vector3&                 m_x_bar,
                                           Matrix3x3& m_x_bar_x_bar)
{
    if(sc.dim() == 3)
    {
        compute_tetmesh_dyadic_mass(sc, m, m_x_bar, m_x_bar_x_bar);
    }
    else if(sc.dim() == 2)
    {
        // UIPC_ASSERT(is_trimesh_closed(sc), "Only closed trimesh is supported.");
        compute_trimesh_dyadic_mass(sc, m, m_x_bar, m_x_bar_x_bar);
    }
    else
    {
        UIPC_ASSERT(false, "Unsupported dimension");
    }
}
}  // namespace uipc::geometry::affine_body
