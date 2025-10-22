#include <uipc/geometry/utils/label_graph_color.h>
#include "graph_coloring/dsatur.hpp"

namespace uipc::geometry
{
S<AttributeSlot<IndexT>> label_graph_color(SimplicialComplex& sc)
{
    GraphColoring::Dsatur algo;
    auto                  node_count = sc.vertices().size();

    vector<GraphColoring::NodeIndexT> Ni;
    Ni.reserve(node_count);
    vector<GraphColoring::NodeIndexT> Nj;
    Nj.reserve(node_count);

    auto edge_view = sc.edges().topo().view();
    for(auto&& edge : edge_view)
    {
        Ni.push_back(edge[0]);
        Nj.push_back(edge[1]);
    }

    algo.build(node_count, Ni, Nj);
    algo.solve();
    auto colors = algo.colors();

    auto color_attr = sc.vertices().find<IndexT>("graph/color");
    if(!color_attr)
        color_attr = sc.vertices().create<IndexT>("graph/color");

    auto color_view = view(*color_attr);
    std::ranges::copy(colors, color_view.begin());

    return color_attr;
}
}  // namespace uipc::geometry
