#include "dsatur.hpp"
#include <uipc/common/log.h>
#include <uipc/common/range.h>
#include <uipc/common/enumerate.h>
#include <algorithm>
#include <iostream>
#include <unordered_set>

namespace GraphColoring
{
using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::map;
using std::ofstream;
using std::ostringstream;
using std::string;
using std::vector;

constexpr auto MinInt = std::numeric_limits<int>::min();

void Dsatur::do_solve()
{
    auto NumNode = this->num_node();
    UIPC_ASSERT(NumNode >= 1, "Graph must have at least one node to color");

    auto node_color = node_colors();
    std::ranges::fill(node_color, ColorIndexT{-1});

    NodeIndexT max_degree_node = 0;
    SizeT      degree          = 0;

    // find maximal degree vertex to color first and color with 0
    for(NodeIndexT i = 0; i < NumNode; ++i)
    {
        if(auto adjs = this->node_adjacency(i); adjs.size() >= degree)
        {
            degree          = adjs.size();
            max_degree_node = i;
        }
    }
    node_color[max_degree_node] = 0;

    // Create saturation_level so that we can see which graph nodes have the
    // highest saturation without having to scan through the entire graph
    // each time
    std::vector<int> saturation_level;
    // Add all nodes and set their saturation level to 0
    saturation_level.resize(NumNode);
    std::ranges::fill(saturation_level, 0);

    // For the single node that has been colored, increment its neighbors so
    // that their current saturation level is correct
    for(const auto max_degree_nodes_adj = this->node_adjacency(max_degree_node);
        const auto Nj : max_degree_nodes_adj)
    {
        saturation_level[Nj] += 1;
    }

    // Set the saturation level of the already completed node to -infinity so
    // that it is not chosen and recolored
    saturation_level[max_degree_node] = MinInt;

    //Populate the todo list with the rest of the vertices that need to be colored
    std::unordered_set<NodeIndexT> todo;
    for(NodeIndexT i = 0; i < NumNode; ++i)
    {
        if(i != max_degree_node)
            todo.insert(i);
    }

    // Color all the remaining nodes in the todo list
    while(!todo.empty())
    {
        // Find the vertex with the highest saturation level
        auto element = std::ranges::max_element(saturation_level);
        NodeIndexT saturation_node = std::distance(saturation_level.begin(), element);
        auto saturation = *element;
        // We now know the most saturated node, so we remove it from the todo list
        todo.erase(saturation_node);

        // Find the highest saturated node and keep its NodeIndex and neighbors colors
        vector<int> saturation_colors;
        auto        saturation_node_adj = this->node_adjacency(saturation_node);
        saturation_colors.resize(saturation_node_adj.size());
        std::ranges::transform(saturation_node_adj,
                               saturation_colors.begin(),
                               [&node_color](const NodeIndexT neighbor)
                               { return node_color[neighbor]; });

        // Find the lowest color that is not being used by any of the most saturated
        // nodes neighbors, then color the most saturated node
        {
            auto sorted_saturation_colors = saturation_colors;
            std::ranges::sort(sorted_saturation_colors);
            auto ret = std::ranges::unique(sorted_saturation_colors);
            sorted_saturation_colors.erase(ret.begin(), ret.end());
            int lowest_color = 0;
            for(const auto c : sorted_saturation_colors)
            {
                if(c == lowest_color)
                {
                    lowest_color += 1;
                }
                else if(c > lowest_color)
                {
                    break;
                }
            }
            node_color[saturation_node] = lowest_color;
        }

        // Since we have colored another node, that nodes neighbors have now
        // become more saturated, so we increase each ones saturation level
        // However we first check that that node has not already been colored
        // (This check is only necessary for enormeous test cases, but is
        // included here for robustness)
        for(const auto Nj : saturation_node_adj)
        {
            if(saturation_level[Nj] != MinInt)
            {
                saturation_level[Nj] += 1;
            }
        }
        saturation_level[saturation_node] = MinInt;
    }
}
}  // namespace GraphColoring