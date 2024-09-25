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


// NOTE:
// To make all allocations in the uipc_core.dll/.so's memory space,
// we need to explicitly instantiate the template function in the .cpp file.
namespace uipc::geometry
{
template <IndexT N>
backend::BufferView backend_view(SimplicialComplexTopo<SimplexSlot<N>>&& v) noexcept
{
    return backend_view(v.m_topology);
}

template <IndexT N>
backend::BufferView backend_view(SimplicialComplexTopo<const SimplexSlot<N>>&& v) noexcept
{
    return backend_view(v.m_topology);
}

template <IndexT N>
span<typename SimplexSlot<N>::ValueT> view(SimplicialComplexTopo<SimplexSlot<N>>&& v)
{
    return view(v.m_topology);
}

template <std::derived_from<ISimplexSlot> SimplexSlot>
auto SimplicialComplexAttributes<SimplexSlot>::topo() noexcept -> Topo
{
    return Topo{m_topology};
}

template <std::derived_from<ISimplexSlot> SimplexSlot>
auto SimplicialComplexAttributes<SimplexSlot>::topo() const noexcept -> ConstTopo
{
    return ConstTopo{m_topology};
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
void SimplicialComplexAttributes<SimplexSlotT>::resize(SizeT size)
    requires(!IsConst)
{
    m_topology.resize(size);
    m_attributes.resize(size);
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
void SimplicialComplexAttributes<SimplexSlotT>::reserve(SizeT size)
    requires(!IsConst)
{
    m_topology.reserve(size);
    m_attributes.reserve(size);
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
void SimplicialComplexAttributes<SimplexSlotT>::clear()
    requires(!IsConst)
{
    m_topology.clear();
    m_attributes.clear();
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
SizeT SimplicialComplexAttributes<SimplexSlotT>::size() const noexcept
{
    return m_attributes.size();
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
void SimplicialComplexAttributes<SimplexSlotT>::destroy(std::string_view name)
    requires(!IsConst)
{
    m_attributes.destroy(name);
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
Json SimplicialComplexAttributes<SimplexSlotT>::to_json() const
{
    Json json = m_attributes.to_json();
    json.push_back(m_topology.to_json());
    return json;
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
SimplicialComplexTopo<SimplexSlotT>::SimplicialComplexTopo(SimplexSlotT& topo)
    : m_topology(topo)
{
}

template <std::derived_from<ISimplexSlot> SimplexSlotT>
void SimplicialComplexTopo<SimplexSlotT>::share(SimplicialComplexTopo<ConstSimplexSlotT>&& topo) && noexcept
    requires(!std::is_const_v<SimplexSlotT>)
{
    UIPC_ASSERT(m_topology.size() == topo.m_topology.size(),
                "Incompatible simplicial complex sizes, input ({}), yours ({}), resize before share",
                topo.m_topology.size(),
                m_topology.size());

    m_topology.share(topo.m_topology);
}

template <std::derived_from<ISimplexSlot> SimplexSlot>
bool SimplicialComplexTopo<SimplexSlot>::is_shared() && noexcept
{
    return m_topology.is_shared();
}

template <std::derived_from<ISimplexSlot> SimplexSlot>
SimplicialComplexAttributes<SimplexSlot>::SimplicialComplexAttributes(
    SimplexSlot& topology, AttributeCollectionT& attributes) noexcept
    : m_topology(topology)
    , m_attributes(attributes)
{
    UIPC_ASSERT(topology.size() == attributes.size(),
                "Topology size({}) and attributes size({}) mismatching",
                topology.size(),
                attributes.size());
}

template class UIPC_CORE_API SimplicialComplexTopo<VertexSlot>;
template class UIPC_CORE_API SimplicialComplexTopo<const VertexSlot>;
template class UIPC_CORE_API SimplicialComplexTopo<SimplexSlot<1>>;
template class UIPC_CORE_API SimplicialComplexTopo<const SimplexSlot<1>>;
template class UIPC_CORE_API SimplicialComplexTopo<SimplexSlot<2>>;
template class UIPC_CORE_API SimplicialComplexTopo<const SimplexSlot<2>>;
template class UIPC_CORE_API SimplicialComplexTopo<SimplexSlot<3>>;
template class UIPC_CORE_API SimplicialComplexTopo<const SimplexSlot<3>>;

template UIPC_CORE_API span<typename SimplexSlot<1>::ValueT> view(
    SimplicialComplexTopo<SimplexSlot<1>>&&);
template UIPC_CORE_API span<typename SimplexSlot<2>::ValueT> view(
    SimplicialComplexTopo<SimplexSlot<2>>&&);
template UIPC_CORE_API span<typename SimplexSlot<3>::ValueT> view(
    SimplicialComplexTopo<SimplexSlot<3>>&&);


template class UIPC_CORE_API SimplicialComplexAttributes<VertexSlot>;
template class UIPC_CORE_API SimplicialComplexAttributes<const VertexSlot>;
template class UIPC_CORE_API SimplicialComplexAttributes<SimplexSlot<1>>;
template class UIPC_CORE_API SimplicialComplexAttributes<const SimplexSlot<1>>;
template class UIPC_CORE_API SimplicialComplexAttributes<SimplexSlot<2>>;
template class UIPC_CORE_API SimplicialComplexAttributes<const SimplexSlot<2>>;
template class UIPC_CORE_API SimplicialComplexAttributes<SimplexSlot<3>>;
template class UIPC_CORE_API SimplicialComplexAttributes<const SimplexSlot<3>>;
}  // namespace uipc::geometry
