#include <uipc/geometry/utils/compute_mesh_area.h>
#include <uipc/common/enumerate.h>
#include <Eigen/Dense>

namespace uipc::geometry
{
UIPC_GEOMETRY_API Float compute_mesh_area(const SimplicialComplex& sc,
                                          Float                    thickness)
{
    UIPC_ASSERT(sc.dim() == 2,
                "compute_mesh_area requires a 2D simplicial complex (triangle mesh), got dim={}.",
                sc.dim());

    auto pos_view = sc.positions().view();
    auto tri_view = sc.triangles().topo().view();

    Float r      = thickness;
    Float volume = 0.0;

    for(auto&& [i, F] : enumerate(tri_view))
    {
        const auto& p0 = pos_view[F[0]];
        const auto& p1 = pos_view[F[1]];
        const auto& p2 = pos_view[F[2]];

        Vector3 e1   = p1 - p0;
        Vector3 e2   = p2 - p0;
        Float   area = 0.5 * e1.cross(e2).norm();

        // effective volume = area * 2r (slab thickness)
        volume += area * 2.0 * r;
    }

    return volume;
}
}  // namespace uipc::geometry

