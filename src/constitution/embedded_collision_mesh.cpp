#include <uipc/constitution/embedded_collision_mesh.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace uipc::constitution
{
// ---------------------------------------------------------------
// CPU helpers — point-in-tet and barycentric coordinates
// ---------------------------------------------------------------

static Float scalar_triple(const Vector3& u,
                            const Vector3& v,
                            const Vector3& w)
{
    return u.dot(v.cross(w));
}

// Compute barycentric coords of p in tet {a,b,c,d}.
// Returns false if the tet is degenerate.
static bool bary_in_tet(const Vector3& p,
                        const Vector3& a,
                        const Vector3& b,
                        const Vector3& c,
                        const Vector3& d,
                        Vector4&       bary)
{
    Float vol = scalar_triple(b - a, c - a, d - a);
    if(std::abs(vol) < Float(1e-12))
        return false;

    Float inv = Float(1) / vol;
    Float l0  = scalar_triple(b - p, c - p, d - p) * inv;
    Float l1  = scalar_triple(a - p, d - p, c - p) * inv;
    Float l2  = scalar_triple(a - p, b - p, d - p) * inv;
    Float l3  = Float(1) - l0 - l1 - l2;

    bary = Vector4(l0, l1, l2, l3);
    return true;
}

static bool is_inside(const Vector4& bary, Float eps = Float(1e-5))
{
    return bary[0] >= -eps && bary[1] >= -eps
        && bary[2] >= -eps && bary[3] >= -eps;
}

// ---------------------------------------------------------------
// EmbeddedCollisionMesh
// ---------------------------------------------------------------

bool EmbeddedCollisionMesh::apply_to(geometry::SimplicialComplex& tet_sc,
                                     geometry::SimplicialComplex& surface_sc) const
{
    auto tet_pos_view  = tet_sc.positions().view();
    auto tets_attr     = tet_sc.tetrahedra().find<Vector4i>(builtin::topo);
    if(!tets_attr)
        return false;
    auto tets_view    = tets_attr->view();
    auto surf_pos_view = surface_sc.positions().view();

    SizeT n_surface = surf_pos_view.size();
    SizeT n_tets    = tets_view.size();

    // Create or reset output attributes on the surface mesh.
    auto tet_index_attr = surface_sc.vertices().find<IndexT>("ecm_tet_index");
    if(!tet_index_attr)
        tet_index_attr =
            surface_sc.vertices().create<IndexT>("ecm_tet_index", -1);

    auto bary_attr = surface_sc.vertices().find<Vector4>("ecm_bary");
    if(!bary_attr)
        bary_attr =
            surface_sc.vertices().create<Vector4>("ecm_bary", Vector4::Zero());

    if(!surface_sc.meta().find<IndexT>("ecm_driven"))
        surface_sc.meta().create<IndexT>("ecm_driven", 1);

    auto tet_index_view = geometry::view(*tet_index_attr);
    auto bary_view      = geometry::view(*bary_attr);

    bool all_embedded = true;

    for(SizeT si = 0; si < n_surface; ++si)
    {
        const Vector3& p = surf_pos_view[si];

        // Brute-force O(n_surface * n_tets) — fine at init.
        // Replace with BVH if tet count > ~10k.
        bool   found         = false;
        Float  best_min_bary = -std::numeric_limits<Float>::max();
        IndexT best_tet      = 0;
        Vector4 best_bary    = Vector4::Constant(Float(0.25));

        for(SizeT ti = 0; ti < n_tets; ++ti)
        {
            const Vector4i& cell = tets_view[ti];
            const Vector3&  a    = tet_pos_view[cell[0]];
            const Vector3&  b    = tet_pos_view[cell[1]];
            const Vector3&  c    = tet_pos_view[cell[2]];
            const Vector3&  d    = tet_pos_view[cell[3]];

            Vector4 bary;
            if(!bary_in_tet(p, a, b, c, d, bary))
                continue;

            if(is_inside(bary))
            {
                tet_index_view[si] = (IndexT)ti;
                bary_view[si]      = bary;
                found              = true;
                break;
            }

            Float min_b = bary.minCoeff();
            if(min_b > best_min_bary)
            {
                best_min_bary = min_b;
                best_tet      = (IndexT)ti;
                best_bary     = bary;
            }
        }

        if(!found)
        {
            // Fallback: clamp to tet boundary and renormalize.
            best_bary     = best_bary.cwiseMax(Float(0));
            Float sum     = best_bary.sum();
            if(sum > Float(1e-12))
                best_bary /= sum;
            else
                best_bary = Vector4::Constant(Float(0.25));

            tet_index_view[si] = best_tet;
            bary_view[si]      = best_bary;
            all_embedded       = false;
        }
    }

    return all_embedded;
}
}
