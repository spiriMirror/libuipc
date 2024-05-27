#include <uipc/common/log.h>

namespace uipc::geometry
{
template <IndexT N>
span<typename SimplexSlot<N>::ValueT> view(SimplicialComplexTopo<SimplexSlot<N>>&& v)
{
    return view(v.m_topology);
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
auto SimplicialComplexAttributes<SimplexSlot>::topo() -> Topo
{
    return m_topology;
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::resize(SizeT size)
{
    m_topology.m_topology.resize(size);
    m_attributes.resize(size);
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::reserve(SizeT size)
{
    m_topology.m_topology.reserve(size);
    m_attributes.reserve(size);
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::clear()
{
    m_topology.m_topology.clear();
    m_attributes.clear();
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
SizeT SimplicialComplexAttributes<SimplexSlot>::size() const
{
    return m_attributes.size();
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::destroy(std::string_view name)
{
    m_attributes.destroy(name);
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
SimplicialComplexTopo<SimplexSlot>::SimplicialComplexTopo(SimplexSlot& topo)
    : m_topology(topo)
{
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
SimplicialComplexAttributes<SimplexSlot>::SimplicialComplexAttributes(SimplexSlot& topology,
                                                                      AttributeCollection& attributes)
    : m_topology(topology)
    , m_attributes(attributes)
{
    UIPC_ASSERT(topology.size() == attributes.size(),
                "Topology size({}) and attributes size({}) mismatching",
                topology.size(),
                attributes.size());
}
}  // namespace uipc::geometry
