#include <muda/compute_graph/compute_graph.h>
#include <muda/compute_graph/nodes/compute_graph_kernel_node.h>
#include <muda/compute_graph/nodes/compute_graph_catpure_node.h>
#include <muda/compute_graph/nodes/compute_graph_memory_node.h>
#include <muda/compute_graph/nodes/compute_graph_event_node.h>
#include <muda/compute_graph/compute_graph_closure.h>
#include <muda/compute_graph/compute_graph_builder.h>

namespace muda
{
namespace details
{
    MUDA_INLINE ComputeGraphAccessor::ComputeGraphAccessor()
        : m_cg(*ComputeGraphBuilder::current_graph())
    {
    }

    MUDA_INLINE void ComputeGraphAccessor::check_allow_var_eval() const
    {
        if(m_cg.m_is_in_capture_func)
            MUDA_ERROR_WITH_LOCATION("you can't eval a var in ComputeGraph::capture() function");
    }

    MUDA_INLINE void ComputeGraphAccessor::check_allow_node_adding() const
    {
        if(m_cg.current_graph_phase() != ComputeGraphPhase::None)
            MUDA_ERROR_WITH_LOCATION("you are not allowed adding node at this point");
    }

    /*
     * Set Graph Node 
     */
    template <typename T>
    MUDA_INLINE void ComputeGraphAccessor::set_kernel_node(const S<KernelNodeParms<T>>& kernelParms)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                MUDA_ASSERT(!kernelParms,
                            "When ComputeGraphPhase == TopoBuilding, "
                            "you don't need to create NodeParms, so keep it nullptr.");
                // fall through
            case ComputeGraphPhase::Building:
                add_kernel_node(kernelParms);
                break;
            case ComputeGraphPhase::Updating:
                update_kernel_node(kernelParms);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }
    template <typename T>
    MUDA_INLINE void ComputeGraphAccessor::add_kernel_node(const S<KernelNodeParms<T>>& parms)
    {
        access_graph([&](Graph& g) {  // create kernel node
            ComputeGraphKernelNode* kernel_node = get_or_create_node<ComputeGraphKernelNode>(
                [&]
                {
                    const auto& [name, closure] = current_closure();
                    return new ComputeGraphKernelNode(NodeId{m_cg.m_nodes.size()},
                                                      m_cg.current_access_index());
                });
            if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
            {
                kernel_node->set_node(g.add_kernel_node(parms));
            }
        });
    }
    template <typename T>
    MUDA_INLINE void ComputeGraphAccessor::update_kernel_node(const S<KernelNodeParms<T>>& kernelParms)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto kernel_node = current_node<ComputeGraphKernelNode>();
                g_exec.set_kernel_node_parms(kernel_node->m_node, kernelParms);
            });
    }

    MUDA_INLINE ComputeGraphAccessor::ComputeGraphAccessor(ComputeGraph& graph)
        : m_cg(graph)
    {
    }

    MUDA_INLINE ComputeGraphAccessor::ComputeGraphAccessor(ComputeGraph* graph)
        : m_cg(*graph)
    {
    }

    MUDA_INLINE auto ComputeGraphAccessor::current_closure() const
        -> const std::pair<std::string, ComputeGraphClosure*>&
    {
        return m_cg.m_closures[m_cg.current_closure_id().value()];
    }

    MUDA_INLINE auto ComputeGraphAccessor::current_closure()
        -> std::pair<std::string, ComputeGraphClosure*>&
    {
        return m_cg.m_closures[m_cg.m_current_closure_id.value()];
    }

    MUDA_INLINE const ComputeGraphNodeBase* ComputeGraphAccessor::current_node() const
    {
        return current_closure().second->m_graph_nodes[m_cg.current_access_index()];
    }

    MUDA_INLINE ComputeGraphNodeBase* ComputeGraphAccessor::current_node()
    {
        return current_closure().second->m_graph_nodes[m_cg.current_access_index()];
    }

    MUDA_INLINE cudaStream_t ComputeGraphAccessor::current_stream() const
    {
        return m_cg.m_current_single_stream;
    }

    MUDA_INLINE cudaStream_t ComputeGraphAccessor::capture_stream() const
    {
        MUDA_ASSERT(m_cg.m_is_capturing, "Not Capture Phase!");
        return m_cg.shared_capture_stream();
    }

    MUDA_INLINE bool ComputeGraphAccessor::is_topo_built() const
    {
        return m_cg.m_is_topo_built;
    }

    template <typename T>
    T* ComputeGraphAccessor::current_node()
    {
        return dynamic_cast<T*>(current_node());
    }

    MUDA_INLINE void ComputeGraphAccessor::set_memcpy_node(void*       dst,
                                                           const void* src,
                                                           size_t size_bytes,
                                                           cudaMemcpyKind kind)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                // fall through
            case ComputeGraphPhase::Building:
                add_memcpy_node(dst, src, size_bytes, kind);
                break;
            case ComputeGraphPhase::Updating:
                update_memcpy_node(dst, src, size_bytes, kind);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::set_memcpy_node(const cudaMemcpy3DParms& parms)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                // fall through
            case ComputeGraphPhase::Building:
                add_memcpy_node(parms);
                break;
            case ComputeGraphPhase::Updating:
                update_memcpy_node(parms);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::set_memset_node(const cudaMemsetParams& parms)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                // fall through
            case ComputeGraphPhase::Building:
                add_memset_node(parms);
                break;
            case ComputeGraphPhase::Updating:
                update_memset_node(parms);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::set_event_record_node(cudaEvent_t event)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                MUDA_ASSERT(!event,
                            "When ComputeGraphPhase == TopoBuilding, "
                            "you don't need to create event, so keep it nullptr.");
                // fall through
            case ComputeGraphPhase::Building:
                add_event_record_node(event);
                break;
            case ComputeGraphPhase::Updating:
                update_event_record_node(event);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::add_memcpy_node(void*       dst,
                                                           const void* src,
                                                           size_t size_bytes,
                                                           cudaMemcpyKind kind)
    {
        access_graph([&](Graph& g) {  // create memory node
            ComputeGraphMemcpyNode* memory_node = get_or_create_node<ComputeGraphMemcpyNode>(
                [&]
                {
                    const auto& [name, closure] = current_closure();
                    return new ComputeGraphMemcpyNode(NodeId{m_cg.m_nodes.size()},
                                                      m_cg.current_access_index());
                });
            if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
                memory_node->set_node(g.add_memcpy_node(dst, src, size_bytes, kind));
        });
    }
    MUDA_INLINE void ComputeGraphAccessor::update_memcpy_node(void*       dst,
                                                              const void* src,
                                                              size_t size_bytes,
                                                              cudaMemcpyKind kind)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto memory_node = current_node<ComputeGraphMemcpyNode>();
                g_exec.set_memcpy_node_parms(memory_node->m_node, dst, src, size_bytes, kind);
            });
    }

    MUDA_INLINE void ComputeGraphAccessor::add_memcpy_node(const cudaMemcpy3DParms& parms)
    {
        access_graph([&](Graph& g) {  // create memory node
            ComputeGraphMemcpyNode* memory_node = get_or_create_node<ComputeGraphMemcpyNode>(
                [&]
                {
                    const auto& [name, closure] = current_closure();
                    return new ComputeGraphMemcpyNode(NodeId{m_cg.m_nodes.size()},
                                                      m_cg.current_access_index());
                });
            if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
                memory_node->set_node(g.add_memcpy_node(parms));
        });
    }

    MUDA_INLINE void ComputeGraphAccessor::update_memcpy_node(const cudaMemcpy3DParms& parms)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto memory_node = current_node<ComputeGraphMemcpyNode>();
                g_exec.set_memcpy_node_parms(memory_node->m_node, parms);
            });
    }

    MUDA_INLINE void ComputeGraphAccessor::add_memset_node(const cudaMemsetParams& parms)
    {
        access_graph([&](Graph& g) {  // create memory node
            ComputeGraphMemsetNode* memory_node = get_or_create_node<ComputeGraphMemsetNode>(
                [&]
                {
                    const auto& [name, closure] = current_closure();
                    return new ComputeGraphMemsetNode(NodeId{m_cg.m_nodes.size()},
                                                      m_cg.current_access_index());
                });
            if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
                memory_node->set_node(g.add_memset_node(parms));
        });
    }

    MUDA_INLINE void ComputeGraphAccessor::update_memset_node(const cudaMemsetParams& parms)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto memory_node = current_node<ComputeGraphMemsetNode>();
                g_exec.set_memset_node_parms(memory_node->m_node, parms);
            });
    }

    MUDA_INLINE void ComputeGraphAccessor::add_event_record_node(cudaEvent_t event)
    {
        MUDA_ASSERT(!m_cg.m_flags.has(muda::GraphInstantiateFlagBit::DeviceLaunch),
                    "Event Record Node is not allowed in a graph that will be launched on device");

        access_graph(
            [&](Graph& g)
            {
                ComputeGraphEventRecordNode* event_record =
                    get_or_create_node<ComputeGraphEventRecordNode>(
                        [&]
                        {
                            const auto& [name, closure] = current_closure();
                            return new ComputeGraphEventRecordNode(
                                NodeId{m_cg.m_nodes.size()}, m_cg.current_access_index());
                        });

                if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
                {
                    event_record->set_node(g.add_event_record_node(event));
                }
            });
    }
    MUDA_INLINE void ComputeGraphAccessor::update_event_record_node(cudaEvent_t event)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto event_record = current_node<ComputeGraphEventRecordNode>();
                g_exec.set_event_record_node_parms(event_record->m_node, event);
            });
    }


    MUDA_INLINE void ComputeGraphAccessor::set_event_wait_node(cudaEvent_t event)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                MUDA_ASSERT(!event,
                            "When ComputeGraphPhase == TopoBuilding, "
                            "you don't need to create event, so keep it nullptr.");
                // fall through
            case ComputeGraphPhase::Building:
                add_event_wait_node(event);
                break;
            case ComputeGraphPhase::Updating:
                update_event_wait_node(event);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::set_capture_node(cudaGraph_t sub_graph)
    {
        switch(ComputeGraphBuilder::current_phase())
        {
            case ComputeGraphPhase::TopoBuilding:
                MUDA_ASSERT(!sub_graph,
                            "When ComputeGraphPhase == TopoBuilding, "
                            "you don't need to create sub_graph, so keep it nullptr.");
            case ComputeGraphPhase::Building:
                add_capture_node(sub_graph);
                break;
            case ComputeGraphPhase::Updating:
                update_capture_node(sub_graph);
                break;
            default:
                MUDA_ERROR_WITH_LOCATION("invalid phase");
                break;
        }
    }

    MUDA_INLINE void ComputeGraphAccessor::add_event_wait_node(cudaEvent_t event)
    {
        MUDA_ASSERT(!m_cg.m_flags.has(muda::GraphInstantiateFlagBit::DeviceLaunch),
                    "Event Wait Node is not allowed in a graph that will be launched on device");

        access_graph(
            [&](Graph& g)
            {
                ComputeGraphEventWaitNode* event_wait =
                    get_or_create_node<ComputeGraphEventWaitNode>(
                        [&]
                        {
                            const auto& [name, closure] = current_closure();
                            return new ComputeGraphEventWaitNode(
                                NodeId{m_cg.m_nodes.size()}, m_cg.current_access_index());
                        });

                if(ComputeGraphBuilder::current_phase() == ComputeGraphPhase::Building)
                {
                    event_wait->set_node(g.add_event_wait_node(event));
                }
            });
    }
    MUDA_INLINE void ComputeGraphAccessor::update_event_wait_node(cudaEvent_t event)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto event_wait = current_node<ComputeGraphEventWaitNode>();
                g_exec.set_event_wait_node_parms(event_wait->m_node, event);
            });
    }

    MUDA_INLINE void ComputeGraphAccessor::add_capture_node(cudaGraph_t sub_graph)
    {
        access_graph(
            [&](Graph& g)
            {
                auto capture_node = get_or_create_node<ComputeGraphCaptureNode>(
                    [&]
                    {
                        const auto& [name, closure] = current_closure();
                        return new ComputeGraphCaptureNode{NodeId{m_cg.m_nodes.size()},
                                                           m_cg.current_access_index()};
                    });
                if(ComputeGraphBuilder::is_building())
                {
                    cudaGraphNode_t node;
                    checkCudaErrors(cudaGraphAddChildGraphNode(
                        &node, g.handle(), nullptr, 0, sub_graph));
                    capture_node->set_node(node);
                    capture_node->update_sub_graph(sub_graph);  // update sub graph
                }
            });
    }

    MUDA_INLINE void ComputeGraphAccessor::update_capture_node(cudaGraph_t sub_graph)
    {
        access_graph_exec(
            [&](GraphExec& g_exec)
            {
                const auto& [name, closure] = current_closure();
                auto capture_node = current_node<ComputeGraphCaptureNode>();
                checkCudaErrors(cudaGraphExecChildGraphNodeSetParams(
                    g_exec.handle(), capture_node->handle(), sub_graph));
                capture_node->update_sub_graph(sub_graph);  // update sub graph
            });
        // m_is_capturing = false;
    }


    template <typename F>
    void ComputeGraphAccessor::access_graph(F&& f)
    {
        f(m_cg.m_graph);
        ++m_cg.m_access_graph_index;
    }

    template <typename F>
    void ComputeGraphAccessor::access_graph_exec(F&& f)
    {
        f(*m_cg.m_graph_exec.get());
    }

    template <typename NodeType, typename F>
    MUDA_INLINE NodeType* ComputeGraphAccessor::get_or_create_node(F&& f)
    {
        static_assert(std::is_base_of_v<ComputeGraphNodeBase, NodeType>,
                      "NodeType must be derived from ComputeGraphNodeBase");
        if(!m_cg.m_is_topo_built)
        {
            NodeType* ptr         = f();
            auto& [name, closure] = current_closure();
            closure->m_graph_nodes.emplace_back(ptr);
            m_cg.m_nodes.emplace_back(ptr);
            return ptr;
        }
        else
            return current_node<NodeType>();
    }
    MUDA_INLINE void ComputeGraphAccessor::set_var_usage(VarId id, ComputeGraphVarUsage usage)
    {
        auto& dst_usage = current_closure().second->m_var_usages[id];
        if(dst_usage < usage)
            dst_usage = usage;
    }

}  // namespace details
}  // namespace muda