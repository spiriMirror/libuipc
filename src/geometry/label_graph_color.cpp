#include <uipc/geometry/utils/label_graph_color.h>
#include "graph_coloring/dsatur.hpp"

namespace uipc::geometry
{
S<AttributeSlot<IndexT>> label_graph_color(SimplicialComplex& sc)
{
    GraphColoring::Dsatur algo;
    auto                  node_count = sc.vertices().size();
    auto                  edge_count = sc.edges().size();

    vector<GraphColoring::NodeIndexT> Ni;
    Ni.reserve(edge_count * 2);
    vector<GraphColoring::NodeIndexT> Nj;
    Nj.reserve(edge_count * 2);

    auto edge_view = sc.edges().topo().view();
    for(auto&& edge : edge_view)
    {
        Ni.push_back(edge[0]);
        Nj.push_back(edge[1]);

        Ni.push_back(edge[1]);
        Nj.push_back(edge[0]);
    }

    algo.build(node_count, Ni, Nj);
    algo.solve();
    UIPC_ASSERT(algo.is_valid(), "invalid graph coloring");
    auto colors = algo.colors();

    auto color_attr = sc.vertices().find<IndexT>("graph/color");
    if(!color_attr)
        color_attr = sc.vertices().create<IndexT>("graph/color");
    auto color_view = view(*color_attr);
    std::ranges::copy(colors, color_view.begin());

    auto color_count_attr = sc.meta().find<IndexT>("graph/color_count");
    if(!color_count_attr)
        color_count_attr = sc.meta().create<IndexT>("graph/color_count");
    auto color_count_view = view(*color_count_attr);
    color_count_view[0] = static_cast<IndexT>(algo.num_color());

    return color_attr;
}
}  // namespace uipc::geometry
