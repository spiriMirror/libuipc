#include "coloring_algorithm.hpp"
#include <uipc/common/log.h>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <ranges>
#include <span>

namespace GraphColoring
{
using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::ostringstream;

void GraphColor::build(SizeT                       node_count,
                       std::span<const NodeIndexT> Ni,
                       std::span<const NodeIndexT> Nj)
{
    m_row_offsets.resize(node_count + 1);
    // sort edges by i
    SizeT                                          edge_count = Ni.size();
    std::vector<std::pair<NodeIndexT, NodeIndexT>> edges(edge_count);

    std::ranges::transform(Ni,
                           Nj,
                           edges.begin(),
                           [node_count](NodeIndexT i, NodeIndexT j)
                           {
                               UIPC_ASSERT(i < node_count && j < node_count,
                                           "Node index out of bounds ({},{}), node count = {}",
                                           i,
                                           j,
                                           node_count);
                               return std::make_pair(i, j);
                           });
    std::ranges::sort(
        edges, std::less<>(), [](const auto& edge) { return edge.first; });

    // make unique edges
    auto ret = std::ranges::unique(edges);
    edges.erase(ret.begin(), ret.end());

    // build row offsets
    for(auto&& i : edges | std::views::keys)
        m_row_offsets[i + 1]++;

    // prefix sum
    std::partial_sum(m_row_offsets.begin(), m_row_offsets.end(), m_row_offsets.begin());

    // build col indices
    m_col_indices.resize(edges.size());
    for(auto&& edge : edges)
    {
        const auto row_offset     = m_row_offsets[edge.first]++;
        m_col_indices[row_offset] = edge.second;
    }
}

std::span<const NodeIndexT> GraphColor::node_adjacency(NodeIndexT node)
{
    const auto row_offset = m_row_offsets[node];
    const auto next_row   = m_row_offsets[node + 1];
    return std::span{m_col_indices}.subspan(row_offset, next_row);
}

SizeT GraphColor::num_node() const
{
    return m_row_offsets.size();
}

std::span<ColorIndexT> GraphColor::node_colors()
{
    return m_graph_colors;
}

std::span<const ColorIndexT> GraphColor::colors() const
{
    return m_graph_colors;
}

void GraphColor::solve()
{
    do_solve();
}

// Checks that no two adjacent nodes have the same color
bool GraphColor::is_valid()
{
    const std::span<const ColorIndexT> color_span = node_colors();
    if(color_span.empty() || num_node() != color_span.size())
    {
        return false;
    }
    for(NodeIndexT Ni = 0; Ni < this->num_node(); ++Ni)
    {
        for(auto Njs = node_adjacency(Ni); auto&& Nj : Njs)
        {
            if(color_span[Ni] == color_span[Nj] || color_span[Ni] == -1)
            {
                return false;
            }
        }
    }
    return true;
}

// Used to print the color of each node in the graph
void GraphColor::report_coloring()
{
    const auto color_span = node_colors();
    std::cout << "Graph Coloring Result: " << endl;
    std::cout << "Node Count: " << num_node() << endl;
    std::cout << "Color Count: " << num_color() << endl;
    for(NodeIndexT i = 0; i < color_span.size(); ++i)
        std::cout << "Node " << i << ": Color " << color_span[i] << endl;
}

SizeT GraphColor::num_color()
{
    return std::ranges::max(node_colors()) + 1;
}

bool GraphColor::is_colored()
{
    auto color_span = node_colors();
    if(color_span.empty())
        return false;
    const auto min_color = std::ranges::min(color_span);
    return min_color >= 0;
}
}  // namespace GraphColoring