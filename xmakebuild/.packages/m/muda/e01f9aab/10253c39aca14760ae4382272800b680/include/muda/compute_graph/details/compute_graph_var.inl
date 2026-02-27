#include <muda/compute_graph/compute_graph_builder.h>
#include <muda/compute_graph/compute_graph_accessor.h>

namespace muda
{
MUDA_INLINE void ComputeGraphVarBase::base_update()
{
    for(auto& [graph, info] : m_related_closure_infos)
    {
        graph->m_need_update = true;
        for(auto& id : info.closure_ids)
            graph->m_closure_need_update[id.value()] = true;
    }
    m_is_valid = true;
}

MUDA_INLINE void ComputeGraphVarBase::base_building_eval()
{
    _building_eval(ComputeGraphVarUsage::ReadWrite);
}

MUDA_INLINE void ComputeGraphVarBase::base_building_ceval() const
{
    _building_eval(ComputeGraphVarUsage::Read);
}

MUDA_INLINE void ComputeGraphVarBase::_building_eval(ComputeGraphVarUsage usage) const
{
    auto acc   = details::ComputeGraphAccessor();
    auto graph = ComputeGraphBuilder::instance().current_graph();
    m_related_closure_infos[graph].closure_ids.insert(graph->current_closure_id());
    graph->emplace_related_var(const_cast<ComputeGraphVarBase*>(this));
    acc.set_var_usage(var_id(), usage);
}

MUDA_INLINE void ComputeGraphVarBase::remove_related_closure_infos(ComputeGraph* graph)
{
    auto iter = m_related_closure_infos.find(graph);
    if(iter != m_related_closure_infos.end())
    {
        m_related_closure_infos.erase(iter);
    }
}

MUDA_INLINE void ComputeGraphVarBase::graphviz_def(std::ostream& o,
                                                   const ComputeGraphGraphvizOptions& options) const
{
    graphviz_id(o, options);
    o << "[";
    if(!name().empty())
        o << "label=\"" << name() << "\",";
    o << options.var_style << "]";
}

MUDA_INLINE void ComputeGraphVarBase::graphviz_id(std::ostream& o,
                                                  const ComputeGraphGraphvizOptions& options) const
{
    o << "var_v" << var_id();
}

MUDA_INLINE void ComputeGraphVarBase::update()
{
    MUDA_ASSERT(!is_using(), "ComputeGraphVar is using, can't update");
    this->base_update();
}

MUDA_INLINE Event::QueryResult ComputeGraphVarBase::query()
{
    for(auto& [graph, info] : m_related_closure_infos)
    {
        if(graph->query() == Event::QueryResult::eNotReady)
            return Event::QueryResult::eNotReady;
    }
    return Event::QueryResult::eFinished;
}

MUDA_INLINE bool ComputeGraphVarBase::is_using()
{
    return query() == Event::QueryResult::eNotReady;
}

MUDA_INLINE void ComputeGraphVarBase::sync()
{
    for(auto& [graph, info] : m_related_closure_infos)
    {
        checkCudaErrors(cudaEventSynchronize(graph->m_event));
    }
}

template <typename RWView>
RWView ComputeGraphVarBase::_eval(const RWView& view)
{
    auto phase = ComputeGraphBuilder::current_phase();
    switch(phase)
    {
        //case ComputeGraphPhase::None: {
        //    MUDA_ERROR_WITH_LOCATION("ComputeGraphVar.eval() is not allowed outside Graph Closure");
        //}
        //break;
        case ComputeGraphPhase::TopoBuilding:
        case ComputeGraphPhase::Building: {
            auto acc = details::ComputeGraphAccessor();
            acc.check_allow_var_eval();
            MUDA_ASSERT(ComputeGraphBuilder::is_topo_building() || is_valid(),
                        "ComputeGraphVar[%s] is not valid, please update it before use",
                        name().data());

            constexpr auto const_eval = is_uniform_view_v<RWView>;

            if constexpr(const_eval)
            {
                // they are all read only(e.g. host float/int ...)
                this->base_building_ceval();
            }
            else
            {
                this->base_building_eval();
            }
        }
        break;
        case ComputeGraphPhase::Updating:
        default:  // nothing to do
            break;
    }
    return view;
}

template <typename ROView>
ROView ComputeGraphVarBase::_ceval(ROView& view) const
{
    auto phase = ComputeGraphBuilder::current_phase();
    switch(phase)
    {
        //case ComputeGraphPhase::None: {
        //    MUDA_ERROR_WITH_LOCATION("ComputeGraphVar.eval() is not allowed outside Graph Closure");
        //}
        //break;
        case ComputeGraphPhase::TopoBuilding:
        case ComputeGraphPhase::Building: {
            auto acc = details::ComputeGraphAccessor();
            acc.check_allow_var_eval();
            MUDA_ASSERT(ComputeGraphBuilder::is_topo_building() || is_valid(),
                        "ComputeGraphVar[%s] is not valid, please update it before use",
                        name().data());

            this->base_building_ceval();
        }
        break;
        case ComputeGraphPhase::Updating: {
            // nothing to do
        }
        default:
            break;
    }
    return view;
}

// ComputeGraphVar<T>:

template <typename T>
MUDA_INLINE void ComputeGraphVar<T>::update(const RWViewer& view)
{
    ComputeGraphVarBase::update();
    m_value = view;
}

template <typename T>
MUDA_INLINE ComputeGraphVar<T>& ComputeGraphVar<T>::operator=(const RWViewer& view)
{
    update(view);
    return *this;
}

template <typename T>
MUDA_INLINE void ComputeGraphVar<T>::graphviz_def(std::ostream& o,
                                                  const ComputeGraphGraphvizOptions& options) const
{
    graphviz_id(o, options);
    o << "[";
    if(!name().empty())
        o << "label=\"" << name() << "\",";

    if constexpr(std::is_same_v<T, cudaEvent_t>)
    {
        o << options.event_style;
    }
    else
    {
        o << options.var_style;
    }

    o << "]";
}
}  // namespace muda