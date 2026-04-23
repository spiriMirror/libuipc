#include <uipc/geometry/utils/graph_partition.h>
#include <uipc/common/log.h>
#include <metis.h>
#include <set>
#include <map>
#include <algorithm>
#include <numeric>

namespace uipc::geometry
{
namespace detail
{
static std::vector<IndexT> graph_partition_impl(SizeT                        n_vertices,
                                                const std::vector<Vector2i>& edges,
                                                const std::vector<IndexT>*   edge_weights,
                                                SizeT                        max_cluster_size)
{
    std::vector<IndexT> result(n_vertices, 0);

    if(n_vertices == 0)
        return result;

    const bool weighted = (edge_weights != nullptr);
    UIPC_ASSERT(!weighted || edge_weights->size() == edges.size(),
                "graph_partition: edge_weights size ({}) must equal edges size ({})",
                weighted ? edge_weights->size() : 0, edges.size());

    // Build CSR adjacency (deduplicate + symmetrize; for weighted, sum duplicates)
    std::vector<std::map<idx_t, idx_t>> neighbors(n_vertices);
    for(SizeT ei = 0; ei < edges.size(); ++ei)
    {
        const auto& e = edges[ei];
        idx_t       a = static_cast<idx_t>(e[0]);
        idx_t       b = static_cast<idx_t>(e[1]);
        if(a >= 0 && b >= 0 && a < static_cast<idx_t>(n_vertices)
           && b < static_cast<idx_t>(n_vertices) && a != b)
        {
            idx_t w = weighted ? static_cast<idx_t>((*edge_weights)[ei]) : 1;
            if(w < 1) w = 1;
            neighbors[a][b] += w;
            neighbors[b][a] += w;
        }
    }

    std::vector<idx_t> xadj(n_vertices + 1);
    std::vector<idx_t> adjncy;
    std::vector<idx_t> adjwgt;
    xadj[0] = 0;
    for(SizeT i = 0; i < n_vertices; ++i)
    {
        for(auto& [n, w] : neighbors[i])
        {
            adjncy.push_back(n);
            if(weighted) adjwgt.push_back(w);
        }
        xadj[i + 1] = static_cast<idx_t>(adjncy.size());
    }

    // Point cloud / isolated vertices: fall back to sequential grouping
    if(adjncy.empty())
    {
        for(SizeT i = 0; i < n_vertices; ++i)
            result[i] = static_cast<IndexT>(i / max_cluster_size);
        return result;
    }

    idx_t n_parts = static_cast<idx_t>((n_vertices + max_cluster_size - 1) / max_cluster_size);

    if(n_parts <= 1)
    {
        // Everything in one partition
        std::fill(result.begin(), result.end(), IndexT{0});
        return result;
    }

    std::vector<idx_t> metis_result(n_vertices, 0);
    bool               success = false;

    idx_t block_size = static_cast<idx_t>(max_cluster_size);
    while(n_parts >= 2 && block_size >= 1)
    {
        idx_t edge_cut   = 0;
        idx_t n_weights  = 1;
        idx_t n_verts    = static_cast<idx_t>(n_vertices);

        idx_t options[METIS_NOPTIONS];
        METIS_SetDefaultOptions(options);
        options[METIS_OPTION_SEED] = 0;  // deterministic

        int ret = METIS_PartGraphKway(&n_verts,
                                      &n_weights,
                                      xadj.data(),
                                      adjncy.data(),
                                      nullptr,
                                      nullptr,
                                      weighted ? adjwgt.data() : nullptr,
                                      &n_parts,
                                      nullptr,
                                      nullptr,
                                      options,
                                      &edge_cut,
                                      metis_result.data());

        if(ret != METIS_OK)
            break;

        // Verify no partition exceeds max_cluster_size
        std::vector<idx_t> sizes(n_parts, 0);
        for(SizeT i = 0; i < n_vertices; ++i)
            sizes[metis_result[i]]++;

        if(*std::max_element(sizes.begin(), sizes.end())
           <= static_cast<idx_t>(max_cluster_size))
        {
            success = true;
            break;
        }

        --block_size;
        n_parts = static_cast<idx_t>((n_vertices + block_size - 1) / block_size);
    }

    if(!success)
    {
        // Sequential fallback
        for(SizeT i = 0; i < n_vertices; ++i)
            metis_result[i] = static_cast<idx_t>(i / max_cluster_size);
    }

    for(SizeT i = 0; i < n_vertices; ++i)
        result[i] = static_cast<IndexT>(metis_result[i]);

    return result;
}
}  // namespace detail

std::vector<IndexT> graph_partition(SizeT                        n_vertices,
                                     const std::vector<Vector2i>& edges,
                                     SizeT                        max_cluster_size)
{
    return detail::graph_partition_impl(n_vertices, edges, nullptr, max_cluster_size);
}

std::vector<IndexT> graph_partition(SizeT                        n_vertices,
                                     const std::vector<Vector2i>& edges,
                                     const std::vector<IndexT>&   edge_weights,
                                     SizeT                        max_cluster_size)
{
    return detail::graph_partition_impl(n_vertices, edges, &edge_weights, max_cluster_size);
}
}  // namespace uipc::geometry
