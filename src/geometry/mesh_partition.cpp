#include <uipc/geometry/utils/mesh_partition.h>
#include <uipc/common/vector.h>
#include <uipc/common/map.h>
#include <uipc/common/range.h>
#include <uipc/common/log.h>
#include <metis.h>
#include <set>
#include <algorithm>
#include <numeric>

namespace uipc::geometry
{
constexpr std::string_view metis_part = "mesh_part";

// Build vertex adjacency CSR from the highest-dimensional simplices
static void build_adjacency(vector<idx_t>&           xadj,
                            vector<idx_t>&           adjncy,
                            SizeT                    vert_count,
                            const SimplicialComplex& sc)
{
    vector<std::set<idx_t>> neighbors(vert_count);

    auto add_edge = [&](IndexT a, IndexT b)
    {
        if(a != b && a >= 0 && b >= 0
           && a < static_cast<IndexT>(vert_count)
           && b < static_cast<IndexT>(vert_count))
        {
            neighbors[a].insert(static_cast<idx_t>(b));
            neighbors[b].insert(static_cast<idx_t>(a));
        }
    };

    // Build from highest-dimensional simplices only
    if(sc.dim() == 3 && sc.tetrahedra().size() > 0)
    {
        auto tet_view = sc.tetrahedra().topo().view();
        for(auto& tet : tet_view)
            for(int i = 0; i < 4; ++i)
                for(int j = i + 1; j < 4; ++j)
                    add_edge(tet[i], tet[j]);
    }
    else if(sc.dim() == 2 && sc.triangles().size() > 0)
    {
        auto tri_view = sc.triangles().topo().view();
        for(auto& tri : tri_view)
            for(int i = 0; i < 3; ++i)
                for(int j = i + 1; j < 3; ++j)
                    add_edge(tri[i], tri[j]);
    }
    else if(sc.dim() == 1 && sc.edges().size() > 0)
    {
        auto edge_view = sc.edges().topo().view();
        for(auto& edge : edge_view)
            add_edge(edge[0], edge[1]);
    }

    // Build CSR format
    xadj.resize(vert_count + 1);
    adjncy.clear();

    xadj[0] = 0;
    for(SizeT i = 0; i < vert_count; ++i)
    {
        for(auto n : neighbors[i])
            adjncy.push_back(n);
        xadj[i + 1] = static_cast<idx_t>(adjncy.size());
    }
}

void mesh_partition(SimplicialComplex& sc, SizeT part_max_size)
{
    SizeT vert_count = sc.vertices().size();

    if(vert_count == 0)
        return;

    auto part_attr = sc.vertices().create<IndexT>(metis_part, -1);
    auto part_view = view(*part_attr);

    // Small mesh: single partition
    if(vert_count <= part_max_size) [[unlikely]]
    {
        std::ranges::fill(part_view, 0);
        return;
    }

    // Build adjacency graph
    vector<idx_t> xadj;
    vector<idx_t> adjncy;
    build_adjacency(xadj, adjncy, vert_count, sc);

    // Point cloud fallback
    if(adjncy.empty())
    {
        for(SizeT i = 0; i < vert_count; ++i)
            part_view[i] = static_cast<IndexT>(i / part_max_size);
        return;
    }

    SizeT block_size = part_max_size;
    idx_t n_parts    = static_cast<idx_t>((vert_count + block_size - 1) / block_size);

    if(n_parts <= 1)
    {
        std::ranges::fill(part_view, 0);
        return;
    }

    // METIS partitioning
    vector<idx_t> metis_result(vert_count, 0);

    bool success = false;
    while(n_parts >= 2 && block_size >= 1)
    {
        idx_t edge_cut   = 0;
        idx_t n_weights  = 1;
        idx_t n_vertices = static_cast<idx_t>(vert_count);

        int ret = METIS_PartGraphKway(&n_vertices,
                                      &n_weights,
                                      xadj.data(),
                                      adjncy.data(),
                                      nullptr,   // vwgt
                                      nullptr,   // vsize
                                      nullptr,   // adjwgt
                                      &n_parts,
                                      nullptr,   // tpwgts
                                      nullptr,   // ubvec
                                      nullptr,   // options
                                      &edge_cut,
                                      metis_result.data());

        if(ret != METIS_OK)
            break;

        // Verify partition sizes
        vector<idx_t> part_sizes(n_parts, 0);
        for(SizeT i = 0; i < vert_count; ++i)
            part_sizes[metis_result[i]]++;

        idx_t max_size = *std::ranges::max_element(part_sizes);
        if(max_size <= static_cast<idx_t>(part_max_size))
        {
            success = true;
            break;
        }

        --block_size;
        n_parts = static_cast<idx_t>((vert_count + block_size - 1) / block_size);
    }

    // Sequential fallback
    if(!success)
    {
        for(SizeT i = 0; i < vert_count; ++i)
            metis_result[i] = static_cast<idx_t>(i / part_max_size);
    }

    // Write to attribute
    for(SizeT i = 0; i < vert_count; ++i)
        part_view[i] = static_cast<IndexT>(metis_result[i]);
}
}  // namespace uipc::geometry
