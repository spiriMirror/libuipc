#include <muda/compute_graph/compute_graph_var_manager.h>
#include <muda/compute_graph/compute_graph.h>

namespace muda
{
MUDA_INLINE span<const ComputeGraphDependency> ComputeGraphClosure::deps() const
{
    return m_graph->dep_span(m_deps_begin, m_deps_count);
}
MUDA_INLINE void ComputeGraphClosure::graphviz_id(std::ostream& o,
                                                  const ComputeGraphGraphvizOptions& options) const
{
    o << "node_g" << options.graph_id << "_n" << clousure_id();
}

MUDA_INLINE void ComputeGraphClosure::graphviz_def(std::ostream& o,
                                                   const ComputeGraphGraphvizOptions& options) const
{
    graphviz_id(o, options);
    o << "[";
    o << "label=\"";

    // "closure | { node1 | node2 | ... }"
    if(!name().empty())
    {
        o << name();
    }
    else
    {
        graphviz_id(o, options);
    }
    if(options.show_all_graph_nodes_in_a_closure)
    {

        o << "|{";
        for(auto& node : m_graph_nodes)
        {
            o << node->name();
            if(&node != &m_graph_nodes.back())
                o << "|";
        }
        o << "}";
    }
    o << "\", ";
    if(options.show_all_graph_nodes_in_a_closure)
    {
        o << options.all_nodes_closure_style;
    }
    else
    {
        o << options.node_style;
    }
    o << "]";
}

MUDA_INLINE void ComputeGraphClosure::graphviz_var_usages(std::ostream& o,
                                                          const ComputeGraphGraphvizOptions& options) const
{
    for(auto&& [var_id, usage] : var_usages())
    {
        auto var = m_graph->m_var_manager->m_vars[var_id.value()];
        var->graphviz_id(o, options);
        o << "->";
        graphviz_id(o, options);
        if(usage == ComputeGraphVarUsage::ReadWrite)
            o << "[" << options.read_write_style << "]";
        else
            o << "[" << options.read_style << "]";
        o << "\n";
    }
}

MUDA_INLINE void ComputeGraphClosure::set_deps_range(size_t begin, size_t count)
{
    m_deps_begin = begin;
    m_deps_count = count;
}
}  // namespace muda