#include <uipc/common/log.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/range.h>
#include <Eigen/Dense>
#include <numbers>

namespace uipc::geometry::affine_body
{
// ref: libuipc/scripts/symbol_calculation/affine_body_quantity.ipynb

static Vector12 compute_tetmesh_body_force(const SimplicialComplex& sc,
                                           const Vector3& body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto    pos_view      = sc.positions().view();
    auto    tet_view      = sc.tetrahedra().topo().view();
    auto    vertex_volume = sc.vertices().find<Float>(builtin::volume);
    Vector3 f             = body_force_density;

    for(auto&& [i, T] : enumerate(tet_view))
    {
        const auto& p0 = pos_view[T[0]];
        const auto& p1 = pos_view[T[1]];
        const auto& p2 = pos_view[T[2]];
        const auto& p3 = pos_view[T[3]];


        Vector3 r0 = p0;
        Vector3 e1 = p1 - p0;
        Vector3 e2 = p2 - p0;
        Vector3 e3 = p3 - p0;

        Float D = e1.dot(e2.cross(e3));

        auto V = D / 6.0;

        auto Q_p =
            [D](IndexT i, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
        {
            Float V = 0.0;

            V += p0(i) / 24;
            V += p1(i) / 24;
            V += p2(i) / 24;
            V += p3(i) / 24;

            return D * V;
        };

        Vector3 Qs = Vector3{
            Q_p(0, p0, p1, p2, p3),  //
            Q_p(1, p0, p1, p2, p3),  //
            Q_p(2, p0, p1, p2, p3)   //
        };

        body_force.segment<3>(0) += f * V;
        body_force.segment<3>(3) += f.x() * Qs;
        body_force.segment<3>(6) += f.y() * Qs;
        body_force.segment<3>(9) += f.z() * Qs;
    }

    return body_force;
}

static Vector12 compute_trimesh_body_force(const SimplicialComplex& sc,
                                           const Vector3& body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto pos_view    = sc.positions().view();
    auto tri_view    = sc.triangles().topo().view();
    auto orient      = sc.triangles().find<IndexT>(builtin::orient);
    auto orient_view = orient ? orient->view() : span<const IndexT>{};

    const auto& f = body_force_density;

    // Using Divergence theorem to compute the body force
    // by integrating on the surface of the trimesh

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 r0 = p0;
        Vector3 e1 = p1 - p0;
        Vector3 e2 = p2 - p0;

        Vector3 N = (p1 - p0).cross(p2 - p0);
        if(orient && orient_view[i] < 0)
            N = -N;

        auto V = p0.dot(N) / 6.0;

        auto Q_p = [](IndexT a, const Vector3& N, const Vector3& p0, const Vector3& p1, const Vector3& p2)
        {
            Float V = 0.0;

            V += p0(a) * p0(a) / 12;
            V += p0(a) * p1(a) / 12;
            V += p0(a) * p2(a) / 12;

            V += p1(a) * p1(a) / 12;
            V += p1(a) * p2(a) / 12;
            V += p2(a) * p2(a) / 12;

            return Float{1.0} / Float{2} * N(a) * V;
        };

        Vector3 Qs = Vector3{
            Q_p(0, N, p0, p1, p2),  //
            Q_p(1, N, p0, p1, p2),  //
            Q_p(2, N, p0, p1, p2)   //
        };

        body_force.segment<3>(0) += f * V;
        body_force.segment<3>(3) += f.x() * Qs;
        body_force.segment<3>(6) += f.y() * Qs;
        body_force.segment<3>(9) += f.z() * Qs;
    }

    return body_force;
}


// ref: libuipc/scripts/symbol_calculation/codim_abd_quantity.ipynb
//      Section "Body Force for Codim ABD"
//
// Derivation: G = int J^T f_density dV.
// Key result: int_slab x^a dV = V_slab * centroid
//   (slab cross-term int_{-r}^{r} s ds = 0, so no centroid shift in normal dir)
//
// Codim shell (open triangle mesh) body force.
// V_i = area_i * 2r,  c_i = (p0+p1+p2)/3
// G[0:3]  += f * V_i
// G[3:12] += f_alpha * V_i * c_i  (each of the 3 affine-row blocks)
//
static Vector12 compute_codim_trimesh_body_force(
    const SimplicialComplex& sc,
    const Vector3&           body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto pos_view = sc.positions().view();
    auto tri_view = sc.triangles().topo().view();

    auto thickness_attr = sc.vertices().find<Float>(builtin::thickness);
    UIPC_ASSERT(thickness_attr,
                "Codim shell body force: `thickness` attribute not found on vertices.");
    Float r    = thickness_attr->view()[0];
    Float twor = 2.0 * r;

    const auto& f = body_force_density;

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 e1   = p1 - p0;
        Vector3 e2   = p2 - p0;
        Float   area = 0.5 * e1.cross(e2).norm();
        Float   V    = area * twor;

        // Centroid of triangle = (p0+p1+p2)/3
        // Qs[j] = V * centroid[j]  (mirrors the tet formula: V * (p0+p1+p2+p3)/4)
        Vector3 Qs = V * (p0 + p1 + p2) / 3.0;

        body_force.segment<3>(0) += f * V;
        body_force.segment<3>(3) += f.x() * Qs;
        body_force.segment<3>(6) += f.y() * Qs;
        body_force.segment<3>(9) += f.z() * Qs;
    }

    return body_force;
}

// ref: libuipc/scripts/symbol_calculation/codim_abd_quantity.ipynb
//      Section "Body Force for Codim ABD"
//
// Derivation: int_cylinder x^a dV = V_cyl * midpoint
//   (disk cross-terms int_disk s1 dA = int_disk s2 dA = 0)
//
// Codim rod (edge mesh) body force.
// V_i = length_i * pi*r^2,  m_i = (p0+p1)/2
// G[0:3]  += f * V_i
// G[3:12] += f_alpha * V_i * m_i  (each of the 3 affine-row blocks)
//
static Vector12 compute_segment_body_force(const SimplicialComplex& sc,
                                           const Vector3& body_force_density)
{
    Vector12 body_force = Vector12::Zero();

    auto pos_view  = sc.positions().view();
    auto edge_view = sc.edges().topo().view();

    auto thickness_attr = sc.vertices().find<Float>(builtin::thickness);
    UIPC_ASSERT(thickness_attr,
                "Codim rod body force: `thickness` attribute not found on vertices.");
    Float r  = thickness_attr->view()[0];
    Float S  = std::numbers::pi_v<Float> * r * r;

    const auto& f = body_force_density;

    for(auto&& [i, E] : enumerate(edge_view))
    {
        const auto& p0 = pos_view[E[0]];
        const auto& p1 = pos_view[E[1]];

        Float length = (p1 - p0).norm();
        Float V      = length * S;

        // Midpoint of segment = (p0+p1)/2
        // Qs[j] = V * midpoint[j]
        Vector3 Qs = V * (p0 + p1) / 2.0;

        body_force.segment<3>(0) += f * V;
        body_force.segment<3>(3) += f.x() * Qs;
        body_force.segment<3>(6) += f.y() * Qs;
        body_force.segment<3>(9) += f.z() * Qs;
    }

    return body_force;
}

UIPC_GEOMETRY_API Vector12 compute_body_force(const SimplicialComplex& sc,
                                              const Vector3& body_force_density)
{
    // Check for codim flag — set by AffineBodyShell / AffineBodyRod
    auto is_codim_attr = sc.meta().find<IndexT>(builtin::is_codim);
    bool codim = is_codim_attr && is_codim_attr->view()[0] == 1;

    if(codim)
    {
        if(sc.dim() == 2)
            return compute_codim_trimesh_body_force(sc, body_force_density);
        else if(sc.dim() == 1)
            return compute_segment_body_force(sc, body_force_density);
        else
            UIPC_ASSERT(false, "Codim body force: unsupported dim {}.", sc.dim());
    }

    if(sc.dim() == 3)
    {
        return compute_tetmesh_body_force(sc, body_force_density);
    }
    else if(sc.dim() == 2)
    {
        return compute_trimesh_body_force(sc, body_force_density);
    }
    else
    {
        UIPC_ASSERT(false, "Only 2D and 3D SimplicialComplex is supported.");
    }
    return Vector12::Zero();
}
}  // namespace uipc::geometry::affine_body
