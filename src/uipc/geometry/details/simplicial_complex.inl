namespace uipc::geometry
{
template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::resize(size_t size)
{
    m_topology->resize(size);
    m_attributes.resize(size);
}
template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::reserve(size_t size)
{
    m_topology->reserve(size);
    m_attributes.reserve(size);
}

template <typename SimplexSlot>
    requires std::is_base_of_v<ISimplexSlot, SimplexSlot>
void SimplicialComplexAttributes<SimplexSlot>::clear()
{
    m_topology->clear();
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
bool SimplicialComplexAttributes<SimplexSlot>::topo_is_owned() const
{
    return m_topology.is_owned();
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
