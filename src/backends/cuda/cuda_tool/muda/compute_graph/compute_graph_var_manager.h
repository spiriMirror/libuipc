#pragma once
#include <driver_types.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <muda/mstl/span.h>
#include <muda/compute_graph/compute_graph_flag.h>
#include <muda/compute_graph/compute_graph_fwd.h>
#include <muda/compute_graph/graphviz_options.h>
namespace muda
{
class ComputeGraphVarManager
{
    template <typename T>
    using S = std::shared_ptr<T>;

  public:
    ComputeGraphVarManager() = default;
    ~ComputeGraphVarManager();

    S<ComputeGraph> create_graph(std::string_view name  = "graph",
                                 ComputeGraphFlag flags = {});


    /**************************************************************
    * 
    * GraphVar API
    * 
    ***************************************************************/
    template <typename T>
    ComputeGraphVar<T>& create_var(std::string_view name);
    template <typename T>
    ComputeGraphVar<T>& create_var(std::string_view name, const T& init_value);
    template <typename T>
    ComputeGraphVar<T>* find_var(std::string_view name);

    bool is_using() const;
    void sync() const;
    void sync_on(cudaStream_t stream) const;

    template <typename... T>
    bool is_using(const ComputeGraphVar<T>&... vars) const;
    template <typename... T>
    void sync(const ComputeGraphVar<T>&... vars) const;
    template <typename... T>
    void sync_on(cudaStream_t stream, const ComputeGraphVar<T>&... vars) const;

    bool is_using(const span<const ComputeGraphVarBase*> vars) const;
    void sync(const span<const ComputeGraphVarBase*> vars) const;
    void sync_on(cudaStream_t stream, const span<const ComputeGraphVarBase*> vars) const;

    const auto& graphs() const { return m_graphs; }
    void graphviz(std::ostream& os, const ComputeGraphGraphvizOptions& options = {}) const;

  private:
    friend class ComputeGraph;
    friend class ComputeGraphNodeBase;
    friend class ComputeGraphClosure;
    std::vector<ComputeGraph*> unique_graphs(span<const ComputeGraphVarBase*> vars) const;
    std::unordered_map<std::string, ComputeGraphVarBase*> m_vars_map;
    std::vector<ComputeGraphVarBase*>                     m_vars;
    std::unordered_set<ComputeGraph*>                     m_graphs;
    span<const ComputeGraphVarBase*>                      var_span() const;
};
}  // namespace muda

#include "details/compute_graph_var_manager.inl"