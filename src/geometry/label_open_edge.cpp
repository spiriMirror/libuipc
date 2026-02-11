#include <uipc/geometry/utils/label_open_edge.h>
#include <uipc/common/enumerate.h>
#include <map>

namespace uipc::geometry
{
UIPC_GEOMETRY_API S<AttributeSlot<IndexT>> label_open_edge(SimplicialComplex& R)
{
    UIPC_ASSERT(R.dim() == 2, "Only 2D SimplicialComplex is supported.");

    auto tri_view = R.triangles().topo().view();
    auto edge_view = R.edges().topo().view();

    // Normalize edge representation: ensure v0 < v1
    auto normalize_edge = [](Vector2i e)
    {
        if(e[0] > e[1])
            std::swap(e[0], e[1]);
        return e;
    };

    // Comparator for Vector2i edges (lexicographic order)
    auto compare_edge = [](const Vector2i& a, const Vector2i& b)
    {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    };

    // Create a map from normalized edge to edge index
    std::map<Vector2i, IndexT, decltype(compare_edge)> edge_to_index(compare_edge);
    for(auto&& [i, e] : enumerate(edge_view))
    {
        Vector2i normalized = normalize_edge(e);
        edge_to_index[normalized] = i;
    }

    // Count how many times each edge appears in triangles
    std::map<Vector2i, int, decltype(compare_edge)> edge_count(compare_edge);

    for(auto&& [i, t] : enumerate(tri_view))
    {
        Vector2i edge0 = normalize_edge({t[0], t[1]});
        Vector2i edge1 = normalize_edge({t[1], t[2]});
        Vector2i edge2 = normalize_edge({t[2], t[0]});

        edge_count[edge0]++;
        edge_count[edge1]++;
        edge_count[edge2]++;
    }

    // Create or find the is_open attribute
    auto is_open_attr = R.edges().find<IndexT>("is_open");
    if(!is_open_attr)
        is_open_attr = R.edges().create<IndexT>("is_open");
    
    auto is_open_view = view(*is_open_attr);

    // Initialize all edges as open (1) - edges not in any triangle are open
    for(auto&& [i, is_open] : enumerate(is_open_view))
    {
        is_open = 1;
    }

    // Set is_open based on edge count
    for(const auto& [edge, count] : edge_count)
    {
        // Check for invalid edges (shared by >=3 triangles)
        UIPC_ASSERT(count < 3,
                    "Edge ({},{}) is shared by {} triangles, which is invalid. "
                    "An edge can only be shared by 1 (open) or 2 (closed) triangles.",
                    edge[0],
                    edge[1],
                    count);

        // Find the edge index
        auto it = edge_to_index.find(edge);
        if(it != edge_to_index.end())
        {
            IndexT edge_idx = it->second;
            // count == 1 means open edge (1), count == 2 means closed edge (0)
            is_open_view[edge_idx] = (count == 1) ? 1 : 0;
        }
    }

    return is_open_attr;
}
}  // namespace uipc::geometry

