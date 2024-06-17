#include <uipc/geometry/simplicial_complex_attributes.h>
#include <uipc/geometry/simplicial_complex_attributes.h>

namespace uipc::geometry
{
UIPC_CORE_API backend::BufferView backend_view(SimplicialComplexTopo<VertexSlot>&& v) noexcept
{
    return backend_view(v.m_topology);
}

span<IndexT> view(SimplicialComplexTopo<VertexSlot>&& v)
{
    return view(v.m_topology);
}

bool SimplicialComplexTopo<VertexSlot>::is_shared() && noexcept
{
    return m_topology.is_shared();
}

SimplicialComplexTopo<VertexSlot>::SimplicialComplexTopo(VertexSlot& v)
    : m_topology{v}
{
}


UIPC_CORE_API backend::BufferView backend_view(SimplicialComplexTopo<const VertexSlot>&& v) noexcept
{
    return backend_view(v.m_topology);
}

bool SimplicialComplexTopo<const VertexSlot>::is_shared() && noexcept
{
    return m_topology.is_shared();
}

void SimplicialComplexTopo<VertexSlot>::share(SimplicialComplexTopo<const VertexSlot>&& v) && noexcept
{
    UIPC_ASSERT(m_topology.size() == v.m_topology.size(),
                "Incompatible simplicial complex sizes, input ({}), yours ({}), resize before share",
                v.m_topology.size(),
                m_topology.size());
    m_topology.share(v.m_topology);
}

SimplicialComplexTopo<const VertexSlot>::SimplicialComplexTopo(const VertexSlot& v)
    : m_topology{v}
{
}
}  // namespace uipc::geometry
