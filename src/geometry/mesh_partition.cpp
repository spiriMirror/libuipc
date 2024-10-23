#include <uipc/geometry/utils/mesh_partition.h>
#include <uipc/common/vector.h>
#include <uipc/common/map.h>
#include <uipc/common/range.h>

namespace uipc::geometry
{
constexpr std::string_view metis_part = "mesh_part";

void mesh_partition(SimplicialComplex& sc, SizeT part_max_size)
{
    //vector<IndexT> xadj;
    //vector<IndexT> adjncy;
    //vector<IndexT> adj_wgt;


    //SizeT        vert_count = sc.vertices().size();
    //auto         part_attr  = sc.vertices().create<IndexT>(metis_part, -1);
    //span<IndexT> part_view  = view(*part_attr);

    //SizeT  block_size = part_max_size;
    //IndexT n_parts    = (vert_count + block_size) / (block_size - 1);

    //if(n_parts == 1) [[unlikely]]
    //{
    //    // no need to partition, all vertices in the same partition
    //    std::ranges::fill(part_view, 0);
    //    return;  // early return
    //}

    //build_xadj(xadj, adjncy, adj_wgt, vert_count, sc.tetrahedra().topo().view());

    //std::vector<IndexT> part_sizes;

    //while(true)
    //{
    //    idx_t egde_cut;
    //    idx_t n_weights  = 1;
    //    idx_t n_vertices = xadj.size() - 1;

    //    int ret = METIS_PartGraphKway(&n_vertices,
    //                                  &n_weights,
    //                                  xadj.data(),
    //                                  adjncy.data(),
    //                                  nullptr,  // vwgt
    //                                  nullptr,  // vsize
    //                                  adj_wgt.data(),
    //                                  &n_parts,
    //                                  nullptr,    // tpwgts
    //                                  nullptr,    // ubvec
    //                                  nullptr,    // options
    //                                  &egde_cut,  // edgecut
    //                                  part_view.data());

    //    UIPC_ASSERT(ret == METIS_OK, "METIS_PartGraphKway failed.");

    //    part_sizes.resize(n_parts);
    //    for(auto i : range(n_vertices))
    //        part_sizes[part_view[i]]++;

    //    IndexT result_max_size = *std::ranges::max_element(part_sizes);

    //    if(result_max_size <= part_max_size)  // the result is acceptable
    //        break;

    //    // otherwise, reduce the block size
    //    --block_size;

    //    UIPC_ASSERT(block_size >= 1, "Unexpected block size, why can it happen?");

    //    // recalculate the n_parts
    //    n_parts = (vert_count + block_size) / (block_size - 1);
    //}
}
}  // namespace uipc::geometry
