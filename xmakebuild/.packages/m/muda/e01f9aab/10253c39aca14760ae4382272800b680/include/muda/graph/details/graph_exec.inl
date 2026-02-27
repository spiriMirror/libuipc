namespace muda
{
MUDA_INLINE GraphExec::GraphExec()
    : m_handle(nullptr)
{
}

MUDA_INLINE GraphExec::GraphExec(GraphExec&& other)
    : m_handle(std::move(other.m_handle))
    , m_cached(std::move(other.m_cached))
{
    other.m_handle = nullptr;
}

MUDA_INLINE GraphExec& GraphExec::operator=(GraphExec&& other)
{
    if(this == &other)
        return *this;
    m_handle       = other.m_handle;
    m_cached       = std::move(other.m_cached);
    other.m_handle = nullptr;
    return *this;
}

MUDA_INLINE void GraphExec::upload(cudaStream_t stream)
{
    checkCudaErrors(cudaGraphUpload(m_handle, stream));
}

MUDA_INLINE void GraphExec::launch(cudaStream_t stream)
{
    checkCudaErrors(cudaGraphLaunch(m_handle, stream));
}

template <typename T>
void GraphExec::set_kernel_node_parms(S<KernelNode> node, const S<KernelNodeParms<T>>& new_parms)
{
    checkCudaErrors(
        cudaGraphExecKernelNodeSetParams(m_handle, node->m_handle, new_parms->handle()));
}

MUDA_INLINE void GraphExec::set_memcpy_node_parms(S<MemcpyNode>  node,
                                                  void*          dst,
                                                  const void*    src,
                                                  size_t         size_bytes,
                                                  cudaMemcpyKind kind)
{
    checkCudaErrors(cudaGraphExecMemcpyNodeSetParams1D(
        m_handle, node->m_handle, dst, src, size_bytes, kind));
}

MUDA_INLINE void GraphExec::set_memcpy_node_parms(S<MemcpyNode> node,
                                                  const cudaMemcpy3DParms& parms)
{
    checkCudaErrors(cudaGraphExecMemcpyNodeSetParams(m_handle, node->m_handle, &parms));
}

MUDA_INLINE void GraphExec::set_memset_node_parms(S<MemsetNode>           node,
                                                  const cudaMemsetParams& parms)
{
    checkCudaErrors(cudaGraphExecMemsetNodeSetParams(m_handle, node->m_handle, &parms));
}

MUDA_INLINE void GraphExec::set_event_record_node_parms(S<EventRecordNode> node, cudaEvent_t event)
{
    checkCudaErrors(cudaGraphExecEventRecordNodeSetEvent(m_handle, node->m_handle, event));
}

MUDA_INLINE void GraphExec::set_event_wait_node_parms(S<EventWaitNode> node, cudaEvent_t event)
{
    checkCudaErrors(cudaGraphExecEventWaitNodeSetEvent(m_handle, node->m_handle, event));
}

MUDA_INLINE GraphViewer GraphExec::viewer() const
{
    return GraphViewer{m_handle, m_flags};
}

MUDA_INLINE GraphExec::~GraphExec()
{
    if(m_handle)
        checkCudaErrors(cudaGraphExecDestroy(m_handle));
}
}  // namespace muda