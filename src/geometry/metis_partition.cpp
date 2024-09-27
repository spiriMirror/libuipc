#include <uipc/geometry/utils/metis_parition.h>
#include <metis.h>
#include <uipc/common/vector.h>
#include <uipc/common/map.h>
#include <uipc/common/range.h>

namespace uipc::geometry
{
constexpr std::string_view metis_part = "metis_part";

static void build_xadj(vector<IndexT>&      xadj,
                       vector<IndexT>&      adjncy,
                       vector<IndexT>&      adj_wgt,
                       SizeT                vert_count,
                       span<const Vector4i> tetrahedra)
{
    std::vector<std::map<IndexT, IndexT>> adj(vert_count);

    auto insert_adj = [&](IndexT i, IndexT j)
    {
        auto iter = adj[i].find(j);
        if(iter == adj[i].end())
        {
            adj[i][j] = 1;
        }
        else
        {
            iter->second += 1;
        }
    };

    xadj.resize(vert_count);

    for(size_t i = 0; i < tetrahedra.size(); i++)
    {
        auto& tet = tetrahedra[i];
        insert_adj(tet.x(), tet.y());
        insert_adj(tet.x(), tet.z());
        insert_adj(tet.x(), tet.w());

        insert_adj(tet.y(), tet.x());
        insert_adj(tet.y(), tet.z());
        insert_adj(tet.y(), tet.w());

        insert_adj(tet.z(), tet.x());
        insert_adj(tet.z(), tet.y());
        insert_adj(tet.z(), tet.w());

        insert_adj(tet.w(), tet.x());
        insert_adj(tet.w(), tet.y());
        insert_adj(tet.w(), tet.z());
    }

    std::transform(adj.begin(),
                   adj.end(),
                   xadj.begin(),
                   [](const std::map<IndexT, int>& s) mutable -> IndexT
                   { return s.size(); });

    // exclusive scan
    std::exclusive_scan(xadj.begin(), xadj.end(), xadj.begin(), 0);

    adjncy.resize(xadj.back());
    adj_wgt.resize(xadj.back());

    for(auto i : range(vert_count))
    {
        std::map<IndexT, IndexT>& adj_i  = adj[i];
        auto                      offset = xadj[i];

        for(auto& [j, weight] : adj_i)
        {
            adjncy[offset]  = j;
            adj_wgt[offset] = weight;
            offset++;
        }
    }
}

void metis_partition(SimplicialComplex& sc, SizeT part_max_size)
{
    vector<IndexT> xadj;
    vector<IndexT> adjncy;
    vector<IndexT> adj_wgt;


    SizeT        vert_count = sc.vertices().size();
    auto         part_attr  = sc.vertices().create<IndexT>(metis_part, -1);
    span<IndexT> part_view  = view(*part_attr);

    SizeT  block_size = part_max_size;
    IndexT n_parts    = (vert_count + block_size) / (block_size - 1);

    if(n_parts == 1) [[unlikely]]
    {
        // no need to partition, all vertices in the same partition
        std::ranges::fill(part_view, 0);
        return;  // early return
    }

    build_xadj(xadj, adjncy, adj_wgt, vert_count, sc.tetrahedra().topo().view());

    std::vector<IndexT> part_sizes;

    while(true)
    {
        idx_t egde_cut;
        idx_t n_weights  = 1;
        idx_t n_vertices = xadj.size() - 1;

        int ret = METIS_PartGraphKway(&n_vertices,
                                      &n_weights,
                                      xadj.data(),
                                      adjncy.data(),
                                      nullptr,  // vwgt
                                      nullptr,  // vsize
                                      adj_wgt.data(),
                                      &n_parts,
                                      nullptr,    // tpwgts
                                      nullptr,    // ubvec
                                      nullptr,    // options
                                      &egde_cut,  // edgecut
                                      part_view.data());

        UIPC_ASSERT(ret == METIS_OK, "METIS_PartGraphKway failed.");

        part_sizes.resize(n_parts);
        for(auto i : range(n_vertices))
            part_sizes[part_view[i]]++;

        IndexT result_max_size = *std::ranges::max_element(part_sizes);

        if(result_max_size <= part_max_size)  // the result is acceptable
            break;

        // otherwise, reduce the block size
        --block_size;

        UIPC_ASSERT(block_size >= 1, "Unexpected block size, why can it happen?");

        // recalculate the n_parts
        n_parts = (vert_count + block_size) / (block_size - 1);
    }
}
}  // namespace uipc::geometry
