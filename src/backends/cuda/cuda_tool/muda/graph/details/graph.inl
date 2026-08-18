namespace muda
{
MUDA_INLINE Graph::Graph()
{
    checkCudaErrors(cudaGraphCreate(&m_handle, 0));
}

MUDA_INLINE Graph::~Graph()
{
    if(m_handle)
        checkCudaErrors(cudaGraphDestroy(m_handle));
}


MUDA_INLINE Graph::Graph(Graph&& o)
    : m_handle(std::move(o.m_handle))
    , m_cached(std::move(o.m_cached))
{
    o.m_handle = nullptr;
}

MUDA_INLINE Graph& Graph::operator=(Graph&& o)
{
    if(this == &o)
        return *this;
    m_handle   = std::move(o.m_handle);
    m_cached   = std::move(o.m_cached);
    o.m_handle = nullptr;
    return *this;
}

MUDA_INLINE auto Graph::instantiate() -> S<GraphExec>
{
    auto ret = std::make_shared<GraphExec>();
    checkCudaErrors(cudaGraphInstantiate(&ret->m_handle, m_handle, nullptr, nullptr, 0));
    return ret;
}

MUDA_INLINE auto Graph::instantiate(Flags<GraphInstantiateFlagBit> flags) -> S<GraphExec>
{
    auto ret = std::make_shared<GraphExec>();
#if MUDA_WITH_DEVICE_STREAM_MODEL
    checkCudaErrors(
        cudaGraphInstantiateWithFlags(&ret->m_handle, m_handle, static_cast<int>(flags)));
#else
    checkCudaErrors(cudaGraphInstantiateWithFlags(
        &ret->m_handle, m_handle, static_cast<int>(flags & GraphInstantiateFlagBit::FreeOnLaunch)));
#endif
    ret->m_flags = flags;
    return ret;
}

template <typename T>
auto Graph::add_kernel_node(const S<KernelNodeParms<T>>& kernelParms,
                            const std::vector<S<GraphNode>>& deps) -> S<KernelNode>
{
    auto                         ret   = std::make_shared<KernelNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddKernelNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), kernelParms->handle()));
    return ret;
}

template <typename T>
auto Graph::add_kernel_node(const S<KernelNodeParms<T>>& kernelParms) -> S<KernelNode>
{
    auto ret = std::make_shared<KernelNode>();
    checkCudaErrors(cudaGraphAddKernelNode(
        &ret->m_handle, m_handle, nullptr, 0, kernelParms->handle()));
    return ret;
}

template <typename T>
auto Graph::add_host_node(const S<HostNodeParms<T>>&       hostParms,
                          const std::vector<S<GraphNode>>& deps) -> S<HostNode>
{
    m_cached.push_back(hostParms);
    auto                         ret   = std::make_shared<HostNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddHostNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), hostParms->handle()));
    return ret;
}

template <typename T>
auto Graph::add_host_node(const S<HostNodeParms<T>>& hostParms) -> S<HostNode>
{
    m_cached.push_back(hostParms);
    auto ret = std::make_shared<HostNode>();
    checkCudaErrors(
        cudaGraphAddHostNode(&ret->m_handle, m_handle, nullptr, 0, hostParms->handle()));
    return ret;
}


MUDA_INLINE auto Graph::add_memcpy_node(void*          dst,
                                        const void*    src,
                                        size_t         size_bytes,
                                        cudaMemcpyKind kind,
                                        const std::vector<S<GraphNode>>& deps) -> S<MemcpyNode>
{
    auto                         ret   = std::make_shared<MemcpyNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddMemcpyNode1D(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), dst, src, size_bytes, kind));
    return ret;
}

MUDA_INLINE auto Graph::add_memcpy_node(void* dst, const void* src, size_t size_bytes, cudaMemcpyKind kind)
    -> S<MemcpyNode>
{
    auto ret = std::make_shared<MemcpyNode>();
    checkCudaErrors(cudaGraphAddMemcpyNode1D(
        &ret->m_handle, m_handle, nullptr, 0, dst, src, size_bytes, kind));
    return ret;
}


MUDA_INLINE auto Graph::add_memcpy_node(const cudaMemcpy3DParms& parms,
                                        const std::vector<S<GraphNode>>& deps) -> S<MemcpyNode>
{
    auto                         ret   = std::make_shared<MemcpyNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddMemcpyNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), &parms));
    return ret;
}

MUDA_INLINE auto Graph::add_memset_node(const cudaMemsetParams& parms,
                                        const std::vector<S<GraphNode>>& deps) -> S<MemsetNode>
{
    auto                         ret   = std::make_shared<MemsetNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddMemsetNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), &parms));
    return ret;
}

MUDA_INLINE auto Graph::add_memset_node(const cudaMemsetParams& parms) -> S<MemsetNode>
{
    auto ret = std::make_shared<MemsetNode>();
    checkCudaErrors(cudaGraphAddMemsetNode(&ret->m_handle, m_handle, nullptr, 0, &parms));
    return ret;
}

MUDA_INLINE auto Graph::add_memcpy_node(const cudaMemcpy3DParms& parms) -> S<MemcpyNode>
{
    auto ret = std::make_shared<MemcpyNode>();
    checkCudaErrors(cudaGraphAddMemcpyNode(&ret->m_handle, m_handle, nullptr, 0, &parms));
    return ret;
}

MUDA_INLINE auto Graph::add_event_record_node(cudaEvent_t e,
                                              const std::vector<S<GraphNode>>& deps)
    -> S<EventRecordNode>
{
    auto                         ret   = std::make_shared<EventRecordNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddEventRecordNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), e));
    return ret;
}

MUDA_INLINE auto Graph::add_event_record_node(cudaEvent_t e) -> S<EventRecordNode>
{
    auto ret = std::make_shared<EventRecordNode>();
    checkCudaErrors(cudaGraphAddEventRecordNode(&ret->m_handle, m_handle, nullptr, 0, e));
    return ret;
}

MUDA_INLINE auto Graph::add_event_wait_node(cudaEvent_t e,
                                            const std::vector<S<GraphNode>>& deps)
    -> S<EventWaitNode>
{
    auto                         ret   = std::make_shared<EventWaitNode>();
    std::vector<cudaGraphNode_t> nodes = map_dependencies(deps);
    checkCudaErrors(cudaGraphAddEventWaitNode(
        &ret->m_handle, m_handle, nodes.data(), nodes.size(), e));
    return ret;
}

MUDA_INLINE auto Graph::add_event_wait_node(cudaEvent_t e) -> S<EventWaitNode>
{
    auto ret = std::make_shared<EventWaitNode>();
    checkCudaErrors(cudaGraphAddEventWaitNode(&ret->m_handle, m_handle, nullptr, 0, e));
    return ret;
}

MUDA_INLINE void Graph::add_dependency(S<GraphNode> from, S<GraphNode> to)
{
#if CUDA_VERSION < 13000
    checkCudaErrors(
        cudaGraphAddDependencies(m_handle, &(from->m_handle), &(to->m_handle), 1));
#else
        checkCudaErrors(
            cudaGraphAddDependencies(m_handle, &(from->m_handle), &(to->m_handle), nullptr /*const cudaGraphEdgeData* edgeData*/, 1));
#endif
}

MUDA_INLINE std::vector<cudaGraphNode_t> Graph::map_dependencies(const std::vector<S<GraphNode>>& deps)
{
    std::vector<cudaGraphNode_t> nodes;
    nodes.reserve(deps.size());
    for(auto d : deps)
        nodes.push_back(d->m_handle);
    return nodes;
}
}  // namespace muda