#include <uipc/geometry/utils/compute_mesh_length.h>
#include <uipc/common/enumerate.h>
#include <numbers>

namespace uipc::geometry
{
UIPC_GEOMETRY_API Float compute_rod_volume(const SimplicialComplex& sc,
                                           Float                    thickness)
{
    UIPC_ASSERT(sc.dim() == 1,
                "compute_mesh_length requires a 1D simplicial complex (edge mesh), got dim={}.",
                sc.dim());

    auto pos_view  = sc.positions().view();
    auto edge_view = sc.edges().topo().view();

    Float r      = thickness;
    Float pi     = std::numbers::pi_v<Float>;
    Float S      = pi * r * r;  // cross-section area
    Float volume = 0.0;

    for(auto&& [i, E] : enumerate(edge_view))
    {
        const auto& p0 = pos_view[E[0]];
        const auto& p1 = pos_view[E[1]];

        Float length = (p1 - p0).norm();

        // effective volume = length * pi * r^2
        volume += length * S;
    }

    return volume;
}
}  // namespace uipc::geometry



