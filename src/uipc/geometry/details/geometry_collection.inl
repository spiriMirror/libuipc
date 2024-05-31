namespace uipc::geometry
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlot<GeometryT>> GeometryCollection::emplace(const GeometryT& geometry)
{
    auto id = m_geometries.size();

    auto slot = std::make_shared<geometry::GeometrySlot<GeometryT>>(id, geometry);
    m_geometries.emplace(id, slot);

    return slot;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlot<GeometryT>> GeometryCollection::find(IndexT id) noexcept
{
    auto it = m_geometries.find(id);
    if(it != m_geometries.end())
    {
        return std::dynamic_pointer_cast<geometry::GeometrySlot<GeometryT>>(it->second);
    }
    return {};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<const geometry::GeometrySlot<GeometryT>> GeometryCollection::find(IndexT id) const noexcept
{
    auto it = m_geometries.find(id);
    if(it != m_geometries.end())
    {
        return std::dynamic_pointer_cast<geometry::GeometrySlot<GeometryT>>(it->second);
    }
    return {};
}
}  // namespace uipc::geometry
