#pragma once
#include <muda/graph/graph_base.h>
#include <muda/graph/kernel_node.h>
#include <muda/graph/memory_node.h>
#include <muda/graph/event_node.h>
#include <muda/graph/graph_viewer.h>

namespace muda
{
class GraphExec
{
    template <typename T>
    using S = std::shared_ptr<T>;
    template <typename T>
    using U = std::unique_ptr<T>;
    cudaGraphExec_t                m_handle;
    Flags<GraphInstantiateFlagBit> m_flags;

  public:
    friend class Graph;

    GraphExec();

    // delete copy
    GraphExec(const GraphExec&)            = delete;
    GraphExec& operator=(const GraphExec&) = delete;

    // move
    GraphExec(GraphExec&& other);
    GraphExec& operator=(GraphExec&& other);

    void upload(cudaStream_t stream = nullptr);

    void launch(cudaStream_t stream = nullptr);

    template <typename T>
    void set_kernel_node_parms(S<KernelNode> node, const S<KernelNodeParms<T>>& new_parms);


    void set_memcpy_node_parms(S<MemcpyNode>  node,
                               void*          dst,
                               const void*    src,
                               size_t         size_bytes,
                               cudaMemcpyKind kind);
    void set_memcpy_node_parms(S<MemcpyNode> node, const cudaMemcpy3DParms& parms);
    void set_memset_node_parms(S<MemsetNode> node, const cudaMemsetParams& parms);


    void set_event_record_node_parms(S<EventRecordNode> node, cudaEvent_t event);
    void set_event_wait_node_parms(S<EventWaitNode> node, cudaEvent_t event);

    ~GraphExec();

    cudaGraphExec_t handle() const { return m_handle; }

    GraphViewer viewer() const;
  private:
    // keep the ref count > 0 for those whose data should be kept alive for the graph life.
    std::list<S<NodeParms>> m_cached;
};
}  // namespace muda

#include "details/graph_exec.inl"