#include <memory>
#include <muda/exception.h>
#include <muda/debug.h>
#include <muda/compute_graph/compute_graph.h>
#include <muda/compute_graph/compute_graph_builder.h>
#include <muda/compute_graph/compute_graph_var.h>
#include <muda/compute_graph/compute_graph_var_manager.h>
#include <muda/compute_graph/compute_graph_node.h>
#include <muda/compute_graph/compute_graph_closure.h>
#include <muda/compute_graph/compute_graph_accessor.h>

namespace muda
{
MUDA_INLINE ComputeGraph::AddNodeProxy::AddNodeProxy(ComputeGraph& cg, std::string_view node_name)
    : m_cg(cg)
    , m_node_name(node_name)
{
}

MUDA_INLINE ComputeGraph::GraphPhaseGuard::GraphPhaseGuard(ComputeGraph& cg, ComputeGraphPhase phase)
    : m_cg(cg)
{
    m_cg.set_current_graph_as_this();
    m_cg.m_current_graph_phase = phase;
}

MUDA_INLINE ComputeGraph::GraphPhaseGuard::~GraphPhaseGuard()
{
    m_cg.m_current_graph_phase = ComputeGraphPhase::None;
    ComputeGraph::clear_current_graph();
}

MUDA_INLINE ComputeGraph& ComputeGraph::AddNodeProxy::operator<<(std::function<void()>&& f) &&
{
    m_cg.add_node(std::move(m_node_name), f);
    return m_cg;
}

MUDA_INLINE void ComputeGraph::graphviz(std::ostream& o, const ComputeGraphGraphvizOptions& options)
{
    topo_build();

    if(options.as_subgraph)
    {
        o << "subgraph cluster_" << options.graph_id;
        o << " {\n";
        o << "label=\"" << name() << "\";\n";
        o << options.cluster_style << "\n";
    }
    else
    {
        o << "digraph G {\n";
        o << options.graph_font << "\n";
    }


    if(options.show_vars && !options.as_subgraph)
    {
        o << "// vars: \n";
        for(auto&& [local_id, var] : m_related_vars)
        {
            var->graphviz_def(o, options);
            o << "\n";
        }
        o << "\n";
    }

    if(options.show_nodes)
    {
        o << "// nodes: \n";
        o << "node_g" << options.graph_id << "[label=\"" << name() << "\""
          << options.node_style << "]\n";

        for(auto& [name, node] : m_closures)
        {
            node->graphviz_def(o, options);
            o << "\n";
        }
        o << "\n";
        if(options.show_vars)
        {
            o << "// node var usages: \n";
            for(auto& [name, node] : m_closures)
            {
                node->graphviz_var_usages(o, options);
                o << "\n";
            }
            o << "\n";
        }
        o << "// node deps: \n";
        for(auto& [name, node] : m_closures)
        {
            if(node->deps().size() != 0)
                continue;
            o << "node_g" << options.graph_id << "->";
            node->graphviz_id(o, options);

            o << "[" << options.arc_style
              << "]"
                 "\n";
        }

        for(auto dep : m_deps)
        {
            auto src = m_closures[dep.to.value()];
            auto dst = m_closures[dep.from.value()];
            dst.second->graphviz_id(o, options);
            o << "->";
            src.second->graphviz_id(o, options);
            o << "[" << options.arc_style
              << "]"
                 "\n";
        }
    }
    o << "}\n";
}

MUDA_INLINE GraphViewer ComputeGraph::viewer()
{
    MUDA_ASSERT(m_graph_exec, "graph is not built yet, call ComputeGraph::build() to build it.");
    return GraphViewer{m_graph_exec->handle(), m_flags};
}

MUDA_INLINE void ComputeGraph::topo_build()
{
    if(m_is_topo_built)
        return;

    m_closure_need_update.clear();
    m_closure_need_update.resize(m_closures.size(), false);
    GraphPhaseGuard guard(*this, ComputeGraphPhase::TopoBuilding);
    for(size_t i = 0; i < m_closures.size(); ++i)
    {
        //m_current_node_id    = NodeId{i};
        m_current_closure_id = ClosureId{i};
        m_allow_access_graph = true;
        m_access_graph_index = 0;
        m_closures[i].second->operator()();
    }

    build_deps();
}

MUDA_INLINE void ComputeGraph::build()
{
    if(m_graph_exec)
        return;

    GraphPhaseGuard guard(*this, ComputeGraphPhase::Building);
    if(!m_is_topo_built)
    {
        m_closure_need_update.clear();
        m_closure_need_update.resize(m_closures.size(), false);
    }
    for(size_t i = 0; i < m_closures.size(); ++i)
    {
        //m_current_node_id    = NodeId{i};
        m_current_closure_id = ClosureId{i};
        m_allow_access_graph = true;
        m_access_graph_index = 0;
        m_closures[i].second->operator()();
    }
    if(!m_is_topo_built)
        build_deps();
    cuda_graph_add_deps();

    m_graph_exec = m_graph.instantiate(m_flags);
    m_graph_exec->upload();
}

MUDA_INLINE void ComputeGraph::serial_launch()
{
    GraphPhaseGuard guard(*this, ComputeGraphPhase::SerialLaunching);

    for(size_t i = 0; i < m_closures.size(); ++i)
    {
        // m_current_node_id    = NodeId{i};
        m_current_closure_id = ClosureId{i};
        m_allow_access_graph = false;  // no need to access graph
        m_closures[i].second->operator()();
        m_is_capturing = false;
    }
}

MUDA_INLINE void ComputeGraph::check_vars_valid()
{
    for(auto&& [local_id, var] : m_related_vars)
        if(!var->is_valid())
        {
            MUDA_ERROR_WITH_LOCATION(
                "var[%s] is not valid, "
                "you need update the var before launch this graph",
                var->name().data());
        }
}

MUDA_INLINE ComputeGraph& ComputeGraph::add_node(std::string&& name,
                                                 const std::function<void()>& f)
{
    details::ComputeGraphAccessor(this).check_allow_node_adding();
    MUDA_ASSERT(m_allow_node_adding,
                "This graph is built or updated, so you can't add new nodes any more.");
    auto size    = m_closures.size();
    auto closure = new ComputeGraphClosure{this, ClosureId{size}, name, f};
    m_closures.emplace_back(name, closure);
    return *this;
}

MUDA_INLINE auto ComputeGraph::dep_span(size_t begin, size_t count) const
    -> span<const Dependency>
{
    return span<const Dependency>{m_deps}.subspan(begin, count);
}

MUDA_INLINE void ComputeGraph::set_current_graph_as_this()
{
    ComputeGraphBuilder::current_graph(this);
}

MUDA_INLINE void ComputeGraph::clear_current_graph()
{
    ComputeGraphBuilder::current_graph(nullptr);
}

MUDA_INLINE Stream& ComputeGraph::shared_capture_stream()
{
    static thread_local Stream s(Stream::Flag::eNonBlocking);
    return s;
}

MUDA_INLINE void ComputeGraph::capture(std::function<void(cudaStream_t)>&& f)
{
    capture("", std::move(f));
}

MUDA_INLINE void ComputeGraph::capture(std::string_view                    name,
                                       std::function<void(cudaStream_t)>&& f)
{
    m_is_in_capture_func = true;

    auto do_capture = [&]
    {
        auto& s = shared_capture_stream();
        // begin capture and pass the stream to f
        m_is_capturing = true;
        s.begin_capture();
        f(s);
        cudaGraph_t g;
        s.end_capture(&g);
        details::ComputeGraphAccessor(this).set_capture_node(g);
        m_is_capturing = false;
    };

    switch(current_graph_phase())
    {
        case ComputeGraphPhase::TopoBuilding:
            // if this is called in topo building phase, we do nothing
            // but just create an empty capture node
            details::LaunchInfoCache::current_capture_name(name);
            details::ComputeGraphAccessor(this).set_capture_node(nullptr);
            details::LaunchInfoCache::current_capture_name("");
            break;
        case ComputeGraphPhase::SerialLaunching: {
            // simply call it
            f(m_current_single_stream);
        }
        break;
        case ComputeGraphPhase::Updating: {
            do_capture();
        }
        break;
        case ComputeGraphPhase::Building: {
            details::LaunchInfoCache::current_capture_name(name);
            do_capture();
            details::LaunchInfoCache::current_capture_name("");
        }
        break;
        default:
            MUDA_ERROR_WITH_LOCATION("invoking capture() outside Graph Closure is not allowed");
            break;
    }
    m_is_in_capture_func = false;
}

MUDA_INLINE ComputeGraphPhase ComputeGraph::current_graph_phase() const
{
    return m_current_graph_phase;
}

MUDA_INLINE void ComputeGraph::_update()
{
    if(!m_need_update)
        return;

    GraphPhaseGuard guard(*this, ComputeGraphPhase::Updating);

    for(size_t i = 0; i < m_closure_need_update.size(); ++i)
    {
        auto& need_update = m_closure_need_update[i];
        if(need_update)
        {
            m_current_closure_id = ClosureId{i};
            // m_current_node_id    = NodeId{i};
            m_allow_access_graph = true;
            m_access_graph_index = 0;
            m_closures[i].second->operator()();
            //if(m_is_capturing)
            //    update_capture_node(m_sub_graphs[i]);
            //m_is_capturing = false;
            need_update = false;
        }
    }
}

MUDA_INLINE ComputeGraph::~ComputeGraph()
{
    for(auto var_info : m_related_vars)
        var_info.var->remove_related_closure_infos(this);

    m_var_manager->m_graphs.erase(this);

    for(auto node : m_nodes)
        delete node;

    for(auto& [name, closure] : m_closures)
        delete closure;
}

MUDA_INLINE void ComputeGraph::emplace_related_var(ComputeGraphVarBase* var)
{
    auto global_var_id = var->var_id();
    auto iter          = m_global_to_local_var_id.find(global_var_id);
    if(iter == m_global_to_local_var_id.end())
    {
        auto local_id = details::LocalVarId{m_related_vars.size()};
        m_related_vars.emplace_back(details::LocalVarInfo{local_id, var});
        m_global_to_local_var_id.emplace(std::make_pair(global_var_id, local_id));
    }
}

MUDA_INLINE ComputeGraph::ComputeGraph(ComputeGraphVarManager& manager,
                                       std::string_view        name,
                                       ComputeGraphFlag        flag)
    : m_var_manager(&manager)
    , m_name(name)
{
    if constexpr(!COMPUTE_GRAPH_ON)
    {
        MUDA_ERROR_WITH_LOCATION("ComputeGraph is disabled, please define MUDA_COMPUTE_GRAPH_ON=1 to enable it.");
    }
    m_var_manager->m_graphs.insert(this);
    switch(flag)
    {
        case ComputeGraphFlag::DeviceLaunch:
            m_flags |= GraphInstantiateFlagBit::DeviceLaunch;
            break;
        default:
            break;
    }
}


MUDA_INLINE ComputeGraph::AddNodeProxy ComputeGraph::create_node(std::string_view node_name)
{
    return AddNodeProxy{*this, node_name};
}

MUDA_INLINE void ComputeGraph::update()
{
    m_allow_node_adding = false;
    check_vars_valid();
    _update();
}

MUDA_INLINE void ComputeGraph::launch(bool single_stream, cudaStream_t s)
{
    m_allow_node_adding = false;
    if(single_stream)
    {
        m_current_single_stream = s;
        serial_launch();
    }
    else
    {
        check_vars_valid();
        build();
        _update();
        m_graph_exec->launch(s);
    }
    m_event_result = Event::QueryResult::eNotReady;
    checkCudaErrors(cudaEventRecord(m_event, s));
#if MUDA_CHECK_ON
    if(Debug::is_debug_sync_all())
        checkCudaErrors(cudaStreamSynchronize(s));
#endif
}

MUDA_INLINE Event::QueryResult ComputeGraph::query() const
{
    if(m_event_result == Event::QueryResult::eNotReady)
        m_event_result = m_event.query();
    return m_event_result;
}
}  // namespace muda

/********************************************************************************
 *
 *                           Build Graph Dependencies
 * 
 ********************************************************************************/
namespace muda
{
namespace details
{
    MUDA_INLINE void process_node(std::vector<ComputeGraph::Dependency>& deps,
                                  std::vector<ClosureId>& last_read_or_write_nodes,
                                  std::vector<ClosureId>& last_write_nodes,
                                  ComputeGraphClosure&    closure,
                                  const std::vector<std::pair<LocalVarId, ComputeGraphVarUsage>>& local_var_usage,
                                  uint64_t& dep_begin,
                                  uint64_t& dep_count)
    {
        auto is_read_write = [](ComputeGraphVarUsage usage)
        { return usage == ComputeGraphVarUsage::ReadWrite; };
        auto is_read_only = [](ComputeGraphVarUsage usage)
        { return usage == ComputeGraphVarUsage::Read; };

        std::unordered_set<ClosureId> unique_deps;

        for(auto& [local_var_id, usage] : local_var_usage)
        {
            // if this is a written resource,
            // this should depend on any write and read before it
            // to get newest data or to avoid data corruption
            if(is_read_write(usage))
            {
                auto dst_nid = last_read_or_write_nodes[local_var_id.value()];
                if(dst_nid.is_valid())
                {
                    // the last accessing node reads or writes this resrouce, so I should depend on it
                    if(unique_deps.find(dst_nid) == unique_deps.end())
                    {
                        // record this dependency
                        unique_deps.insert(dst_nid);
                    }
                }
            }
            // if this is a read resource,
            // this should depend on any write before it
            // to get newest data
            // but it has no need to depend on any read before it
            else if(is_read_only(usage))
            {
                auto dst_nid = last_write_nodes[local_var_id.value()];
                if(dst_nid.is_valid())
                {
                    // the last accessing node writes this resrouce, so I should depend on it
                    if(unique_deps.find(dst_nid) == unique_deps.end())
                    {
                        // record this dependency
                        unique_deps.insert(dst_nid);
                    }
                }
            }
        }

        auto current_closure_id = closure.clousure_id();

        // set up res node map with pair [res, node]
        for(auto& [local_var_id, usage] : local_var_usage)
        {
            // if this is a write resource,
            // the latter read/write kernel should depend on this
            // to get the newest data.
            if(is_read_write(usage))
            {
                last_read_or_write_nodes[local_var_id.value()] = current_closure_id;
                last_write_nodes[local_var_id.value()] = current_closure_id;
            }
            // if this is a read resource,
            // the latter write kernel should depend on this
            // to avoid data corruption.
            else if(is_read_only(usage))
            {
                last_read_or_write_nodes[local_var_id.value()] = current_closure_id;
            }
        }

        // add dependencies to deps
        dep_begin = deps.size();
        for(auto dep : unique_deps)
            deps.emplace_back(ComputeGraph::Dependency{dep, current_closure_id});
        dep_count = unique_deps.size();
    }
}  // namespace details

MUDA_INLINE void ComputeGraph::cuda_graph_add_deps()
{
    std::vector<cudaGraphNode_t> froms;
    froms.reserve(m_deps.size());
    std::vector<cudaGraphNode_t> tos;
    tos.reserve(m_deps.size());

    // in closure deps

    for(auto& [name, closure] : m_closures)
    {
        MUDA_ASSERT(closure->m_graph_nodes.size() > 0, "closure[%s] has no nodes", name.data());
        if(closure->m_graph_nodes.size() == 1)
            continue;

        auto from = closure->m_graph_nodes.front();
        auto to   = from;
        for(size_t i = 1; i < closure->m_graph_nodes.size(); ++i)
        {
            to = closure->m_graph_nodes[i];
            froms.emplace_back(from->handle());
            tos.emplace_back(to->handle());
            from = to;
        }
    }


    for(auto dep : m_deps)
    {
        auto from = m_closures[dep.from.value()].second->m_graph_nodes.back();
        auto to   = m_closures[dep.to.value()].second->m_graph_nodes.front();
        froms.emplace_back(from->handle());
        tos.emplace_back(to->handle());
    };
#if CUDA_VERSION < 13000
    checkCudaErrors(cudaGraphAddDependencies(
        m_graph.handle(), froms.data(), tos.data(), froms.size()));
#else
    checkCudaErrors(cudaGraphAddDependencies(
        m_graph.handle(), froms.data(), tos.data(), nullptr /*const cudaGraphEdgeData* edgeData*/, froms.size()));
#endif
}

MUDA_INLINE void ComputeGraph::build_deps()
{
    m_deps.clear();
    auto local_var_count = m_related_vars.size();

    // map: var_id -> node_id, uint64_t{-1} means no write node yet
    auto last_write_nodes = std::vector<ClosureId>(local_var_count, ClosureId{});
    // map: var_id -> node_id, uint64_t{-1} means no read node yet
    auto last_read_or_write_nodes = std::vector<ClosureId>(local_var_count, ClosureId{});

    // process all nodes
    for(size_t i = 0u; i < m_closures.size(); i++)
    {
        auto& [name, closure] = m_closures[i];

        // map global var id to local var id
        std::vector<std::pair<details::LocalVarId, ComputeGraphVarUsage>> local_var_usage;
        local_var_usage.reserve(closure->var_usages().size());
        for(auto&& [var_id, usage] : closure->var_usages())
        {
            auto local_id = m_global_to_local_var_id[var_id];
            local_var_usage.emplace_back(local_id, usage);
        }

        size_t dep_begin, dep_count;
        details::process_node(
            m_deps, last_read_or_write_nodes, last_write_nodes, *closure, local_var_usage, dep_begin, dep_count);
        closure->set_deps_range(dep_begin, dep_count);
    }

    m_is_topo_built = true;
}
}  // namespace muda
