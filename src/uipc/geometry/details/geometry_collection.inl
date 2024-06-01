namespace uipc::geometry
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlot<GeometryT>> GeometryCollection::emplace(const GeometryT& geometry)
{
    auto id = m_next_id++;

    auto slot = std::make_shared<geometry::GeometrySlot<GeometryT>>(id, geometry);
    m_geometries.emplace(id, slot);

    return slot;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlot<GeometryT>> GeometryCollection::pending_emplace(const GeometryT& geometry)
{
    auto id = m_next_id++;

    auto slot = std::make_shared<geometry::GeometrySlot<GeometryT>>(id, geometry);
    m_pending_create.emplace(id, slot);

    return slot;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlot<GeometryT>> GeometryCollection::find(IndexT id) noexcept
{
    if(auto it = m_pending_destroy.find(id); it != m_pending_destroy.end())
    {
        return {};
    }

    if(auto it = m_geometries.find(id); it != m_geometries.end())
    {
        return std::dynamic_pointer_cast<geometry::GeometrySlot<GeometryT>>(it->second);
    }

    if(auto it = m_pending_create.find(id); it != m_pending_create.end())
    {
        return std::dynamic_pointer_cast<geometry::GeometrySlot<GeometryT>>(it->second);
    }
    return {};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<const geometry::GeometrySlot<GeometryT>> GeometryCollection::find(IndexT id) const noexcept
{
    return const_cast<GeometryCollection*>(this)->find<GeometryT>(id);
}
}  // namespace uipc::geometry
