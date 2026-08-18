#pragma once
#include <functional>
#include <map>
#include <muda/compute_graph/compute_graph_node_type.h>
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/compute_graph/compute_graph_node_id.h>
#include <muda/compute_graph/compute_graph_var_usage.h>
#include <muda/compute_graph/compute_graph_var_id.h>
#include <muda/compute_graph/graphviz_options.h>
#include <muda/compute_graph/compute_graph_dependency.h>

namespace muda
{
class ComputeGraphClosure
{
    friend class details::ComputeGraphAccessor;

  public:
    auto        clousure_id() const { return m_clousure_id; }
    auto        type() const { return m_type; }
    auto        name() const { return std::string_view{m_name}; }
    const auto& var_usages() const { return m_var_usages; }
    span<const ComputeGraphDependency> deps() const;

    virtual void graphviz_id(std::ostream& o, const ComputeGraphGraphvizOptions& options) const;
    virtual void graphviz_def(std::ostream& o, const ComputeGraphGraphvizOptions& options) const;
    virtual void graphviz_var_usages(std::ostream& o,
                                     const ComputeGraphGraphvizOptions& options) const;
    virtual ~ComputeGraphClosure() = default;

  protected:
    template <typename T>
    using S = std::shared_ptr<T>;

    friend class ComputeGraph;
    friend class ComputeGraphVarBase;
    ComputeGraphClosure(ComputeGraph*               graph,
                        ClosureId                   clousure_id,
                        std::string_view            name,
                        const std::function<void()> f)
        : m_graph(graph)
        , m_clousure_id(clousure_id)
        , m_name(name)
        , m_closure(f)
    {
    }

    std::function<void()>                 m_closure;
    std::map<VarId, ComputeGraphVarUsage> m_var_usages;
    ClosureId                             m_clousure_id;
    uint64_t                              m_access_graph_index;
    ComputeGraph*                         m_graph;
    std::string                           m_name;
    ComputeGraphNodeType                  m_type;
    size_t                                m_deps_begin = 0;
    size_t                                m_deps_count = 0;

    void operator()() { m_closure(); }

    std::vector<ComputeGraphNodeBase*> m_graph_nodes;
    void set_deps_range(size_t begin, size_t count);
};
}  // namespace muda

#include "details/compute_graph_closure.inl"