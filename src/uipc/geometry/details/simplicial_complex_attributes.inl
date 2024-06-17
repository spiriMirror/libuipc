#include <uipc/common/log.h>

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
}  // namespace uipc::geometry
