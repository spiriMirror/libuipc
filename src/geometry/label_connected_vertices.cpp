#include <uipc/geometry/utils/label_connected_vertices.h>
#include <uipc/common/enumerate.h>
#include <igl/connected_components.h>

namespace uipc::geometry
{
S<AttributeSlot<IndexT>> label_connected_vertices(SimplicialComplex& complex)
{
    IndexT N_vert = complex.vertices().size();

    auto region = complex.vertices().find<IndexT>("region");
    if(!region)
        region = complex.vertices().create<IndexT>("region");
    auto region_view = view(*region);

    std::vector<Eigen::Triplet<IndexT>> triplets{complex.edges().size() * 2};
    auto edge_view = complex.edges().topo().view();

    for(auto [i, edge] : enumerate(edge_view))
    {
        UIPC_ASSERT(edge[0] != edge[1],
                    "Self-loop is not allowed. In edge[{}] = ({},{})",
                    i,
                    edge[0],
                    edge[1]);

        triplets[i * 2]     = {edge[0], edge[1], 1};
        triplets[i * 2 + 1] = {edge[1], edge[0], 1};
    }

    Eigen::SparseMatrix<IndexT> adjacency_matrix{N_vert, N_vert};
    adjacency_matrix.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXi C;
    Eigen::VectorXi K;

    auto N_region = igl::connected_components(adjacency_matrix, C, K);

    // fill the region attribute
    for(auto&& [i, r] : enumerate(region_view))
        r = C[i];

    auto region_count = complex.meta().find<IndexT>("region_count");
    if(!region_count)
        region_count = complex.meta().create<IndexT>("region_count");
    auto region_count_view = view(*region_count);
    region_count_view[0]   = N_region;

    return region;
}
}  // namespace uipc::geometry
