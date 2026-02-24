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

// Build the CSR adjacency graph (xadj, adjncy) from the simplicial complex topology.
static void build_adjacency(vector<idx_t>&           xadj,
                            vector<idx_t>&           adjncy,
                            SizeT                    vert_count,
                            const SimplicialComplex& sc)
{
    // Collect all edges from all simplex types
    vector<std::set<IndexT>> neighbors(vert_count);

    auto add_edge = [&](IndexT a, IndexT b)
    {
        if(a != b)
        {
            neighbors[a].insert(b);
            neighbors[b].insert(a);
        }
    };

    // From tetrahedra (dim=3): 6 edges per tet
    if(sc.dim() >= 3 && sc.tetrahedra().size() > 0)
    {
        auto tet_view = sc.tetrahedra().topo().view();
        for(auto& tet : tet_view)
        {
            for(int i = 0; i < 4; ++i)
                for(int j = i + 1; j < 4; ++j)
                    add_edge(tet[i], tet[j]);
        }
    }

    // From triangles (dim=2): 3 edges per tri
    if(sc.dim() >= 2 && sc.triangles().size() > 0)
    {
        auto tri_view = sc.triangles().topo().view();
        for(auto& tri : tri_view)
        {
            for(int i = 0; i < 3; ++i)
                for(int j = i + 1; j < 3; ++j)
                    add_edge(tri[i], tri[j]);
        }
    }

    // From edges (dim=1): 1 edge per element
    if(sc.dim() >= 1 && sc.edges().size() > 0)
    {
        auto edge_view = sc.edges().topo().view();
        for(auto& edge : edge_view)
        {
            add_edge(edge[0], edge[1]);
        }
    }

    // Build CSR format
    xadj.resize(vert_count + 1);
    adjncy.clear();

    xadj[0] = 0;
    for(SizeT i = 0; i < vert_count; ++i)
    {
        for(auto n : neighbors[i])
            adjncy.push_back(static_cast<idx_t>(n));
        xadj[i + 1] = static_cast<idx_t>(adjncy.size());
    }
}

void mesh_partition(SimplicialComplex& sc, SizeT part_max_size)
{
    SizeT vert_count = sc.vertices().size();

    if(vert_count == 0)
        return;

    auto  part_attr = sc.vertices().create<IndexT>(metis_part, -1);
    auto  part_view = view(*part_attr);

    // If the mesh is small enough, put all vertices in partition 0
    if(vert_count <= part_max_size) [[unlikely]]
    {
        std::ranges::fill(part_view, 0);
        return;
    }

    // Build adjacency graph from the simplicial complex
    vector<idx_t> xadj;
    vector<idx_t> adjncy;
    build_adjacency(xadj, adjncy, vert_count, sc);

    // If no edges (point cloud), assign sequential partitions
    if(adjncy.empty())
    {
        for(SizeT i = 0; i < vert_count; ++i)
            part_view[i] = static_cast<IndexT>(i / part_max_size);
        return;
    }

    SizeT  block_size = part_max_size;
    idx_t  n_parts    = static_cast<idx_t>((vert_count + block_size - 1) / block_size);

    if(n_parts <= 1)
    {
        std::ranges::fill(part_view, 0);
        return;
    }

    // Use a temporary buffer for METIS output (idx_t may differ from IndexT)
    vector<idx_t> metis_part_result(vert_count);

    while(true)
    {
        idx_t edge_cut;
        idx_t n_weights  = 1;
        idx_t n_vertices = static_cast<idx_t>(vert_count);

        int ret = METIS_PartGraphKway(&n_vertices,
                                      &n_weights,
                                      xadj.data(),
                                      adjncy.data(),
                                      nullptr,  // vwgt
                                      nullptr,  // vsize
                                      nullptr,  // adjwgt
                                      &n_parts,
                                      nullptr,  // tpwgts
                                      nullptr,  // ubvec
                                      nullptr,  // options
                                      &edge_cut,
                                      metis_part_result.data());

        UIPC_ASSERT(ret == METIS_OK,
                     "METIS_PartGraphKway failed with return code {}.",
                     ret);

        // Check if all partitions are within the size limit
        vector<idx_t> part_sizes(n_parts, 0);
        for(SizeT i = 0; i < vert_count; ++i)
            part_sizes[metis_part_result[i]]++;

        idx_t result_max_size = *std::ranges::max_element(part_sizes);

        if(result_max_size <= static_cast<idx_t>(part_max_size))
            break;

        // Reduce effective block size and retry
        --block_size;
        UIPC_ASSERT(block_size >= 1, "Unexpected block size during mesh partition, why can it happen?");
        n_parts = static_cast<idx_t>((vert_count + block_size - 1) / block_size);
    }

    // Copy result to the attribute
    for(SizeT i = 0; i < vert_count; ++i)
        part_view[i] = static_cast<IndexT>(metis_part_result[i]);
}
}  // namespace uipc::geometry
