#include <uipc/geometry/simplicial_complex_attributes.h>

namespace uipc::geometry
{
backend::BufferView backend_view(SimplicialComplexTopo<VertexSlot>&& v) noexcept
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


backend::BufferView backend_view(SimplicialComplexTopo<const VertexSlot>&& v) noexcept
{
    return backend_view(v.m_topology);
}

bool SimplicialComplexTopo<const VertexSlot>::is_shared() && noexcept
{
    return m_topology.is_shared();
}

SimplicialComplexTopo<const VertexSlot>::SimplicialComplexTopo(const VertexSlot& v)
    : m_topology{v}
{
}
}  // namespace uipc::geometry
