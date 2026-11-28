#include <uipc/geometry/utils/is_trimesh_closed.h>

namespace uipc::geometry
{
UIPC_GEOMETRY_API bool is_trimesh_closed(const SimplicialComplex& R)
{
    UIPC_ASSERT_THROW(R.dim() == 2, "Only 2D SimplicialComplex is supported.");

    auto pos_view = R.positions().view();
    auto tri_view = R.triangles().topo().view();

    if(tri_view.size() % 2)  // odd number of triangles
        return false;

    vector<Vector2i> tri_edges(3 * tri_view.size());

    auto swap_if_necessary = [](Vector2i e)
    {
        if(e[0] > e[1])
            std::swap(e[0], e[1]);
        return e;
    };

    for(auto&& [i, t] : enumerate(tri_view))
    {
        tri_edges[3 * i + 0] = swap_if_necessary({t[0], t[1]});
        tri_edges[3 * i + 1] = swap_if_necessary({t[1], t[2]});
        tri_edges[3 * i + 2] = swap_if_necessary({t[2], t[0]});
    }

    std::ranges::sort(tri_edges,
                      [](Vector2i a, Vector2i b)
                      { return a[0] < b[0] || a[0] == b[0] && a[1] < b[1]; });

    const auto edge_count = tri_edges.size();
    for(SizeT i = 0; i < edge_count; i += 2)
    {
        const auto& edge = tri_edges[i];
        if(edge != tri_edges[i + 1])
            return false;
        if(i + 2 < edge_count && edge == tri_edges[i + 2])
            return false;
    }

    return true;
}
}  // namespace uipc::geometry
