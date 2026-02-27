#pragma once
#include <map>
#include <functional>
#include <set>
#include <muda/launch/stream.h>
#include <muda/launch/event.h>
#include <muda/mstl/span.h>
#include <muda/graph/graph.h>
#include <muda/graph/graph_viewer.h>
#include <muda/compute_graph/compute_graph_flag.h>
#include <muda/compute_graph/compute_graph_phase.h>
#include <muda/compute_graph/compute_graph_node_type.h>
#include <muda/compute_graph/compute_graph_node_id.h>
#include <muda/compute_graph/compute_graph_closure_id.h>
#include <muda/compute_graph/compute_graph_var_id.h>
#include <muda/compute_graph/compute_graph_var_usage.h>
#include <muda/compute_graph/compute_graph_dependency.h>
#include <muda/compute_graph/graphviz_options.h>
#include <muda/compute_graph/compute_graph_fwd.h>

namespace muda
{
namespace details
{
    class LocalVarId : public U64IdWithType
    {
        using U64IdWithType::U64IdWithType;
    };
    class LocalVarInfo
    {
      public:
        LocalVarId           id{};
        ComputeGraphVarBase* var = nullptr;
    };
}  // namespace details

class ComputeGraph
{
  public:
    class AddNodeProxy
    {
        ComputeGraph& m_cg;
        std::string   m_node_name;

      public:
        AddNodeProxy(ComputeGraph& cg, std::string_view node_name);
        ComputeGraph& operator<<(std::function<void()>&& f) &&;
    };
    // A depends on B : from B to A
    using Dependency = ComputeGraphDependency;

    class GraphPhaseGuard
    {
        ComputeGraph& m_cg;

      public:
        GraphPhaseGuard(ComputeGraph& cg, ComputeGraphPhase phase);
        ~GraphPhaseGuard();
    };

    // delete copy
    ComputeGraph(const ComputeGraph&)            = delete;
    ComputeGraph& operator=(const ComputeGraph&) = delete;

    // delete move
    ComputeGraph(ComputeGraph&&)            = delete;
    ComputeGraph& operator=(ComputeGraph&&) = delete;

  private:
    //class TempNodeInfo
    //{
    //  public:
    //    std::map<VarId, ComputeGraphVarUsage> var_usage;
    //};
    template <typename T>
    using U = std::unique_ptr<T>;
    template <typename T>
    using S = std::shared_ptr<T>;

    friend class ComputeGraphVarBase;

    Graph        m_graph;
    S<GraphExec> m_graph_exec{nullptr};

    std::unordered_map<NodeId::value_type, cudaGraph_t> m_sub_graphs;

    std::vector<std::pair<std::string, ComputeGraphClosure*>> m_closures;

    std::map<VarId, details::LocalVarId> m_global_to_local_var_id;
    std::vector<details::LocalVarInfo>   m_related_vars;
    void emplace_related_var(ComputeGraphVarBase* var);


    std::vector<ComputeGraphNodeBase*>              m_nodes;
    std::vector<std::vector<ComputeGraphNodeBase*>> m_graph_nodes;
    std::vector<Dependency>                         m_deps;

    std::vector<int>        m_closure_need_update;
    ComputeGraphVarManager* m_var_manager = nullptr;

    friend class ComputeGraphVarManager;

    Event                      m_event;
    mutable Event::QueryResult m_event_result = Event::QueryResult::eFinished;
    Flags<GraphInstantiateFlagBit> m_flags;

  public:
    ComputeGraph(ComputeGraphVarManager& manager,
                 std::string_view        name = "graph",
                 ComputeGraphFlag        flag = ComputeGraphFlag::HostLaunch);

    ~ComputeGraph();

    /**************************************************************
    * 
    * Info API
    * 
    ***************************************************************/

    std::string_view name() const { return m_name; }

    /**************************************************************
    * 
    * GraphNode API
    * 
    ***************************************************************/

    AddNodeProxy create_node(std::string_view node_name);


    /**************************************************************
    * 
    * Graph Launch API
    * 
    ***************************************************************/

    void update();

    void build();

    void launch(bool single_stream, cudaStream_t s = nullptr);

    void launch(cudaStream_t s = nullptr) { return launch(false, s); }

    /**************************************************************
    * 
    * Graph Event Query API
    * 
    ***************************************************************/

    Event::QueryResult query() const;

    /**************************************************************
    * 
    * Graph Closure Capture Node API
    * 
    ***************************************************************/

    void capture(std::function<void(cudaStream_t)>&& f);
    void capture(std::string_view name, std::function<void(cudaStream_t)>&& f);

    /**************************************************************
    * 
    * Graph Visualization API
    * 
    ***************************************************************/

    void graphviz(std::ostream& o, const ComputeGraphGraphvizOptions& options = {});

    /**************************************************************
    * 
    * Graph Viewer API
    * 
    ***************************************************************/

    GraphViewer viewer();

    operator GraphViewer() { return viewer(); }

  private:  // internal method
    void topo_build();

    void cuda_graph_add_deps();

    void build_deps();

    void serial_launch();

    void _update();

    void check_vars_valid();

    friend class AddNodeProxy;
    ComputeGraph& add_node(std::string&& name, const std::function<void()>& f);

    friend class ComputeGraphNodeBase;
    friend class ComputeGraphClosure;
    span<const Dependency> dep_span(size_t begin, size_t count) const;

    void set_current_graph_as_this();

    static void clear_current_graph();

    static Stream& shared_capture_stream();

    friend class ComputeGraphBuilder;
    ClosureId current_closure_id() const { return m_current_closure_id; };

    NodeId current_node_id() const { return m_current_node_id; };

    size_t current_access_index() const { return m_access_graph_index; }

    ComputeGraphPhase current_graph_phase() const;

  private:  // internal data
    friend class muda::details::ComputeGraphAccessor;
    std::string       m_name;
    bool              m_need_update = false;
    ClosureId         m_current_closure_id;
    NodeId            m_current_node_id;
    ComputeGraphPhase m_current_graph_phase = ComputeGraphPhase::None;
    bool              m_allow_access_graph  = false;
    size_t            m_access_graph_index  = 0;
    bool              m_allow_node_adding   = true;
    // TempNodeInfo      m_temp_node_info;
    cudaStream_t m_current_single_stream = nullptr;
    bool         m_is_capturing          = false;
    // in capture func, we don't allow any var eval()
    bool m_is_in_capture_func = false;
    // if we have already built the topo, we don't do that again
    bool m_is_topo_built = false;
};
}  // namespace muda

#include "details/compute_graph.inl"