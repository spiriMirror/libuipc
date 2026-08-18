#pragma once
#include <unordered_map>
#include <unordered_set>

#include <muda/graph/graph_base.h>
#include <muda/graph/graph_exec.h>

#include <muda/graph/kernel_node.h>
#include <muda/graph/memory_node.h>
#include <muda/graph/host_node.h>
#include <muda/graph/event_node.h>

#include <muda/graph/graph_instantiate_flag.h>

namespace muda
{
class Graph
{
    template <typename T>
    using S = std::shared_ptr<T>;
    template <typename T>
    using U = std::unique_ptr<T>;

  public:
    Graph();
    ~Graph();

    // delete copy
    Graph(const Graph&)            = delete;
    Graph& operator=(const Graph&) = delete;

    // move
    Graph(Graph&&);
    Graph& operator=(Graph&&);


    friend class GraphExec;
    friend class std::shared_ptr<Graph>;

    MUDA_NODISCARD S<GraphExec> instantiate();
    MUDA_NODISCARD S<GraphExec> instantiate(Flags<GraphInstantiateFlagBit> flags);

    template <typename T>
    S<KernelNode> add_kernel_node(const S<KernelNodeParms<T>>&     kernelParms,
                                  const std::vector<S<GraphNode>>& deps);
    template <typename T>
    S<KernelNode> add_kernel_node(const S<KernelNodeParms<T>>& kernelParms);


    template <typename T>
    S<HostNode> add_host_node(const S<HostNodeParms<T>>&       hostParms,
                              const std::vector<S<GraphNode>>& deps);
    template <typename T>
    S<HostNode> add_host_node(const S<HostNodeParms<T>>& hostParms);


    S<MemcpyNode> add_memcpy_node(void*                            dst,
                                  const void*                      src,
                                  size_t                           size_bytes,
                                  cudaMemcpyKind                   kind,
                                  const std::vector<S<GraphNode>>& deps);
    S<MemcpyNode> add_memcpy_node(void* dst, const void* src, size_t size_bytes, cudaMemcpyKind kind);
    S<MemcpyNode> add_memcpy_node(const cudaMemcpy3DParms& parms);
    S<MemcpyNode> add_memcpy_node(const cudaMemcpy3DParms&         parms,
                                  const std::vector<S<GraphNode>>& deps);

    S<MemsetNode> add_memset_node(const cudaMemsetParams&          parms,
                                  const std::vector<S<GraphNode>>& deps);
    S<MemsetNode> add_memset_node(const cudaMemsetParams& parms);


    S<EventRecordNode> add_event_record_node(cudaEvent_t e,
                                             const std::vector<S<GraphNode>>& deps);
    S<EventRecordNode> add_event_record_node(cudaEvent_t e);
    S<EventWaitNode>   add_event_wait_node(cudaEvent_t                      e,
                                           const std::vector<S<GraphNode>>& deps);
    S<EventWaitNode>   add_event_wait_node(cudaEvent_t e);


    void add_dependency(S<GraphNode> from, S<GraphNode> to);

    cudaGraph_t handle() const { return m_handle; }
    cudaGraph_t handle() { return m_handle; }
    static auto create() { return std::make_shared<Graph>(); }

  private:
    cudaGraph_t m_handle;
    // keep the ref count > 0 for those whose data should be kept alive for the graph life.
    std::list<S<NodeParms>> m_cached;
    static std::vector<cudaGraphNode_t> map_dependencies(const std::vector<S<GraphNode>>& deps);
};
}  // namespace muda

#include "details/graph.inl"