#pragma once
#include <muda/graph/graph_base.h>

namespace muda
{
#ifdef MUDA_WITH_GRAPH_MEMORY_ALLOC_FREE
class MemAllocNode : public GraphNode
{
    void* m_dptr;

  public:
    using this_type = MemAllocNode;
    friend class Graph;
};

template <typename T>
class MemAllocNodeParms : public NodeParms
{
    cudaMemAllocNodeParams m_parms;

  public:
    using this_type = MemAllocNodeParms;
    friend class graph;
    friend class std::shared_ptr<this_type>;
    friend class std::unique_ptr<this_type>;
    friend class std::weak_ptr<this_type>;

    MemAllocNodeParms(size_t size)
        : m_parms({})
    {
        m_parms.poolProps.allocType = cudaMemAllocationTypePinned;
        cudaGetDevice(&m_parms.poolProps.location.id);
        m_parms.poolProps.location.type = cudaMemLocationTypeDevice;
        m_parms.bytesize                = size * sizeof(T);
    }

    cudaMemAllocNodeParams*       getRaw() { return &m_parms; }
    const cudaMemAllocNodeParams* getRaw() const { return &m_parms; }
};

class MemFreeNode : public GraphNode
{
  public:
    using this_type = MemFreeNode;
    friend class Graph;
};
#endif

class MemcpyNode : public GraphNode
{
  public:
    using this_type = MemcpyNode;
    friend class Graph;
};

class MemsetNode : public GraphNode
{
  public:
    using this_type = MemsetNode;
    friend class Graph;
};
}  // namespace muda