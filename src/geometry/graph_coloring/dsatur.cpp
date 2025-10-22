#include "dsatur.hpp"
#include <uipc/common/log.h>
#include <uipc/common/range.h>
#include <uipc/common/enumerate.h>
#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <set>

namespace GraphColoring
{
using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::map;
using std::ofstream;
using std::ostringstream;
using std::set;
using std::string;
using std::vector;

// ref: https://www.geeksforgeeks.org/dsa/dsatur-algorithm-for-graph-coloring/
// A C++ program to implement the DSatur algorithm for graph
// coloring

using namespace std;

// Struct to store information
// on each uncoloured vertex
class NodeInfo
{
  public:
    int64_t sat;     // Saturation degree of the vertex
    int64_t deg;     // Degree in the uncoloured subgraph
    int64_t vertex;  // Index of vertex
};

class MaxSatOp
{
  public:
    bool operator()(const NodeInfo& lhs, const NodeInfo& rhs) const
    {
        // Compares two nodes by
        // saturation degree, then
        // degree in the subgraph,
        // then vertex label
        return tie(lhs.sat, lhs.deg, lhs.vertex) > tie(rhs.sat, rhs.deg, rhs.vertex);
    }
};

// Assigns colors (starting from 0)
// to all vertices and
// prints the assignment of colors
void Dsatur::do_solve()
{
    // Output: node colors
    auto node_colors = this->node_colors();

    const auto              n = this->num_node();
    vector<int>             color_usage(n, 0);  // flag
    vector<int64_t>         node_degrees(n);
    vector<set<int64_t>>    adj_colors(n);
    set<NodeInfo, MaxSatOp> Q;

    // Initialise the data structures.
    // These are a (binary tree) priority queue, a set of colours adjacent to each uncoloured vertex (initially empty)
    // and the degree d(v) of each uncoloured vertex in the graph induced by uncoloured vertices
    for(int64_t u = 0; u < n; u++)
    {
        node_colors[u]  = -1ll;
        auto adj_u      = this->node_adjacency(u);
        node_degrees[u] = static_cast<int64_t>(adj_u.size());
        adj_colors[u]   = set<int64_t>();
        Q.emplace(NodeInfo{0, node_degrees[u], u});
    }

    while(!Q.empty())
    {
        // Choose the vertex u with the highest saturation degree, breaking ties with degree
        const auto    maxPtr = Q.begin();
        const int64_t u      = maxPtr->vertex;
        // Remove u from the priority queue
        Q.erase(maxPtr);

        // Identify the lowest feasible
        span<const NodeIndexT> adj_u = this->node_adjacency(u);
        // used colour for vertex u
        for(const NodeIndexT& v : adj_u)
        {
            if(const auto node_color_v = node_colors[v]; node_color_v != -1)
                color_usage[node_color_v] = 1;
        }

        // Find the first unused colour
        const auto unused_color_iter = std::ranges::find(color_usage, 0);
        const auto unused_color_i =
            static_cast<int64_t>(std::distance(color_usage.begin(), unused_color_iter));
        UIPC_ASSERT(unused_color_i >= 0, "There should be at least one unused color");

        // Reset the color usage for the next iteration
        for(auto&& v : adj_u)
        {
            if(const auto node_color_v = node_colors[v]; node_color_v != -1)
                color_usage[node_color_v] = 0;
        }


        // Assign vertex u to unused_color
        node_colors[u] = unused_color_i;

        // Update the saturation degrees and
        // degrees of all uncoloured neighbours;
        // hence modify their corresponding
        // elements in the priority queue
        for(auto&& v : adj_u)
        {
            if(node_colors[v] == -1)
            {
                auto& adj_v_colors   = adj_colors[v];
                auto& node_v_degrees = node_degrees[v];
                // Remove the old entry of v
                Q.erase(NodeInfo{static_cast<int64_t>(adj_v_colors.size()), node_v_degrees, v});

                // Update the set of colours
                adj_v_colors.insert(unused_color_i);
                node_v_degrees--;
                Q.emplace(NodeInfo{static_cast<int64_t>(adj_v_colors.size()), node_v_degrees, v});
            }
        }
    }
}
}  // namespace GraphColoring
