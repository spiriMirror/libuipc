#include <uipc/geometry/utils/compute_proper_d_hat.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/zip.h>

namespace uipc::geometry
{
S<AttributeSlot<Float>> compute_proper_d_hat(SimplicialComplex& R)
{
    if(R.dim() == 0)
    {
        spdlog::warn("Can't compute proper d_hat on a point cloud, skip.");
        return nullptr;
    }

    if(R.edges().size() == 0)
    {
        spdlog::warn("Can't compute proper d_hat on a mesh without edges, skip.");
        return nullptr;
    }

    auto edge_is_surf = R.edges().find<IndexT>(builtin::is_surf);
    if(!edge_is_surf)
    {
        spdlog::warn("Can't compute proper d_hat on a mesh without `is_surf` attribute, did you forget to call `label_surface()`?");
        return nullptr;
    }

    auto edge_is_surf_view = edge_is_surf->view();
    bool any_surf =
        std::ranges::any_of(edge_is_surf_view,
                            [](IndexT is_surf) { return is_surf > 0; });
    if(!any_surf)
    {
        spdlog::warn("Can't compute proper d_hat on a mesh without surface edge, skip.");
        return nullptr;
    }

    auto pos_view = R.positions().view();

    // D_hat = d_hat * d_hat
    Float min_D_hat = std::numeric_limits<Float>::max();

    auto d_hat_attr = R.meta().create<Float>("d_hat");
    auto d_hat_view = view(*d_hat_attr);


    if(R.dim() >= 1)  // linemesh/trimesh/tetmesh
    {
        // using min edge length as d_hat
        auto edge_view = R.edges().topo().view();
        for(auto&& [e, is_surf] : zip(edge_view, edge_is_surf_view))
        {
            auto& p0    = pos_view[e[0]];
            auto& p1    = pos_view[e[1]];
            Float D_hat = (p1 - p0).squaredNorm();
            if(is_surf)
                min_D_hat = std::min(min_D_hat, D_hat);
        }

        d_hat_view[0] = std::sqrt(min_D_hat);
    }

    return d_hat_attr;
}
}  // namespace uipc::geometry
