#pragma once
#include <string>
namespace muda
{
class ComputeGraphGraphvizOptions
{
  public:
    bool show_vars                         = true;
    bool show_nodes                        = true;
    bool as_subgraph                       = false;
    bool show_all_graph_nodes_in_a_closure = false;
    int  graph_id                          = 0;

    // styles
    std::string node_style =
        R"(shape="egg", color="#82B366", style="filled", fillcolor="#D5E8D4",)";

    std::string all_nodes_closure_style =
        R"(shape="Mrecord", color="#82B366", style="filled", fillcolor="#D5E8D4",)";

    std::string var_style =
        R"(shape="rectangle", color="#F08705", style="filled,rounded", fillcolor="#F5AF58",)";

    std::string read_write_style = R"(color="#F08E81", arrowhead = diamond,)";

    std::string read_style = R"(color="#64BBE2", arrowhead = dot, )";

    std::string arc_style = R"(color="#82B366", )";

    std::string event_style =
        R"(shape="rectangle", color="#8E44AD", style="filled,rounded", fillcolor="#BB8FCE",)";

    std::string graph_viewer_style =
        R"(shape="rectangle", color="#82B366", style="filled,rounded", fillcolor="#D5E8D4",)";

    std::string cluster_style =
        R"(fontcolor="#82B366" fontsize=18; color = "#82B366"; style = "dashed";)";

    std::string cluster_var_style = R"(color="#F08705"; style="dashed";)";

    std::string graph_font = R"(graph [fontname = "helvetica"];
node [fontname = "helvetica"];
edge [fontname = "helvetica"];)";
};
}  // namespace muda