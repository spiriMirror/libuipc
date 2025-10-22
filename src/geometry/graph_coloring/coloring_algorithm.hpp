#pragma once
#include <cinttypes>
#include <map>
#include <string>
#include <vector>
#include <span>

namespace GraphColoring
{
using std::map;
using std::string;
using std::vector;
using NodeIndexT  = uint64_t;
using ColorIndexT = int64_t;
using SizeT       = uint64_t;

class GraphColor
{
  private:
    // CSR representation of the graph
    // row_offsets.size() == num_nodes + 1
    std::vector<SizeT> m_row_offsets;
    // For Node i:
    //    Adjs = col_indices[row_offsets[i] ... row_offsets[i+1]-1]
    std::vector<NodeIndexT> m_col_indices;
    // Output color
    std::vector<ColorIndexT> m_graph_colors;

  protected:
    std::span<ColorIndexT> node_colors();

  public:
    /* Constructors */
    GraphColor()          = default;
    virtual ~GraphColor() = default;
    /**
     * @brief Build from Edges
     * @param node_count Total Node Count
     * @param Ni First Node of Edge
     * @param Nj Second Node of Edge
     */
    void build(SizeT                       node_count,
               std::span<const NodeIndexT> Ni,  //
               std::span<const NodeIndexT> Nj);

    void solve();

    std::span<const NodeIndexT>  node_adjacency(NodeIndexT node) const;
    SizeT                        num_node() const;
    SizeT                        num_color();
    std::span<const ColorIndexT> colors() const;
    bool                         is_colored();
    bool                         is_valid() const;
    /* Print functions */
    void report_coloring();

  private:
    virtual void do_solve() = 0;
};
}  // namespace GraphColoring
