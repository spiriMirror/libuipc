#pragma once
#include <functional>
#include <muda/compute_graph/compute_graph_phase.h>
#include <muda/compute_graph/compute_graph_fwd.h>
#include <functional>

namespace muda
{
class ComputeGraphBuilder
{
    static ComputeGraphBuilder& instance();
    using Phase         = ComputeGraphPhase;
    using PhaseAction   = std::function<void()>;
    using CaptureAction = std::function<void(cudaStream_t)>;

  public:
    static Phase current_phase();
    static void  capture(CaptureAction&& cap);
    static void  capture(std::string_view name, CaptureAction&& cap);
    static bool  is_phase_none();
    static bool  is_phase_serial_launching();
    static bool  is_topo_building();
    static bool  is_building();
    // return true when no graph is building or the graph is in serial launching mode
    static bool is_direct_launching();
    static bool is_caturing();


    // do_when_direct_launch
    // do_when_set_node => do_when_add_node & do_when_update_node
    // if do_when_topo_building_set_node == nullptr, do_when_set_node will be called
    // if do_when_topo_building_set_node != nullptr, do_when_topo_building_set_node will be called
    // copy this code to use:
    /*
            ComputeGraphBuilder::invoke_phase_actions(
            [&] // do_when_direct_launch
            {

            },
            [&] // do_when_set_node
            {

            },
            [&] // do_when_topo_building_set_node
            {

            });
    */
    static void invoke_phase_actions(PhaseAction&& do_when_direct_launch,
                                     PhaseAction&& do_when_set_node,
                                     PhaseAction&& do_when_topo_building_set_node);

    // copy this code to use:
    /*
            ComputeGraphBuilder::invoke_phase_actions(
            [&] // do_when_direct_launch
            {

            },
            [&] // do_when_set_node and do_when_topo_building_set_node
            {

            });
    */
    static void invoke_phase_actions(PhaseAction&& do_when_direct_launch,
                                     PhaseAction&& do_when_set_node);

    // copy this code to use:
    /*
            ComputeGraphBuilder::invoke_phase_actions(
            [&] // do_in_every_phase
            {

            });
    */
    static void invoke_phase_actions(PhaseAction&& do_in_every_phase);

  private:
    friend class ComputeGraph;
    friend class ComputeGraphVarBase;

    static void current_graph(ComputeGraph* graph);
    friend class details::ComputeGraphAccessor;
    static auto current_graph() { return instance().m_current_graph; }

    ComputeGraphBuilder()  = default;
    ~ComputeGraphBuilder() = default;

    ComputeGraph* m_current_graph = nullptr;
};
}  // namespace muda

#include "details/compute_graph_builder.inl"