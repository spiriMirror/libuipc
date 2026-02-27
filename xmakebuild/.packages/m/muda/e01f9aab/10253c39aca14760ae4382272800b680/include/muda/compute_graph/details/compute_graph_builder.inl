#include <muda/compute_graph/compute_graph.h>

namespace muda
{
MUDA_INLINE ComputeGraphPhase ComputeGraphBuilder::current_phase()
{
    auto ins = instance().m_current_graph;
    if(ins)
        return ins->current_graph_phase();
    else
        return ComputeGraphPhase::None;
}

MUDA_INLINE void ComputeGraphBuilder::capture(CaptureAction&& cap)
{
    MUDA_ASSERT(instance().current_graph(), "Error current graph = nullptr!");
    instance().current_graph()->capture(std::move(cap));
}

MUDA_INLINE void ComputeGraphBuilder::capture(std::string_view name, CaptureAction&& cap)
{
    MUDA_ASSERT(instance().current_graph(), "Error current graph = nullptr!");
    instance().current_graph()->capture(name, std::move(cap));
}

MUDA_INLINE bool ComputeGraphBuilder::is_phase_none()
{
    return current_phase() == ComputeGraphPhase::None;
}

MUDA_INLINE bool ComputeGraphBuilder::is_phase_serial_launching()
{
    return current_phase() == ComputeGraphPhase::SerialLaunching;
}

MUDA_INLINE bool ComputeGraphBuilder::is_topo_building()
{
    return current_phase() == ComputeGraphPhase::TopoBuilding;
}

MUDA_INLINE bool ComputeGraphBuilder::is_building()
{
    return current_phase() == ComputeGraphPhase::Building;
}

MUDA_INLINE bool ComputeGraphBuilder::is_direct_launching()
{
    return is_phase_serial_launching() || is_phase_none();
}

MUDA_INLINE bool ComputeGraphBuilder::is_caturing()
{
    return is_building() && instance().m_current_graph->m_is_capturing;
}

MUDA_INLINE void ComputeGraphBuilder::invoke_phase_actions(PhaseAction&& do_when_direct_launching,
                                                           PhaseAction&& do_when_set_node,
                                                           PhaseAction&& do_when_topo_building)
{
    MUDA_ASSERT(do_when_direct_launching, "do_when_direct_launching is null");
    MUDA_ASSERT(do_when_set_node, "do_when_set_node is null");
    MUDA_ASSERT(do_when_topo_building, "do_when_topo_building is null");
    if(is_direct_launching())
        do_when_direct_launching();
    else if(is_building())
        do_when_set_node();
    else if(is_topo_building())
        do_when_topo_building();
}

MUDA_INLINE void ComputeGraphBuilder::invoke_phase_actions(PhaseAction&& do_when_direct_launching,
                                                           PhaseAction&& do_when_set_node)
{
    MUDA_ASSERT(do_when_direct_launching, "do_when_direct_launching is null");
    MUDA_ASSERT(do_when_set_node, "do_when_set_node is null");

    if(is_direct_launching())
        do_when_direct_launching();
    else if(is_building() || is_topo_building())
        do_when_set_node();
}

MUDA_INLINE void ComputeGraphBuilder::invoke_phase_actions(PhaseAction&& do_in_every_phase)
{
    if(is_direct_launching() || is_building() || is_topo_building())
        do_in_every_phase();
}

MUDA_INLINE void ComputeGraphBuilder::current_graph(ComputeGraph* graph)
{
    instance().m_current_graph = graph;
}

MUDA_INLINE ComputeGraphBuilder& muda::ComputeGraphBuilder::instance()
{
    thread_local static ComputeGraphBuilder builder;
    return builder;
}
}  // namespace muda