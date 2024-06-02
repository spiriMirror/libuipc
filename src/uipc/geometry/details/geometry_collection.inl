namespace uipc::geometry
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlotT<GeometryT>> GeometryCollection::emplace(const GeometryT& geometry)
{
    m_dirty = true;

    auto id = m_next_id++;

    auto slot = std::make_shared<geometry::GeometrySlotT<GeometryT>>(id, geometry);
    slot->state(geometry::GeometrySlotState::Normal);
    m_geometries.emplace(id, slot);

    return slot;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
P<geometry::GeometrySlotT<GeometryT>> GeometryCollection::pending_emplace(const GeometryT& geometry)
{
    m_dirty = true;

    auto id = m_next_id++;

    auto slot = std::make_shared<geometry::GeometrySlotT<GeometryT>>(id, geometry);
    slot->state(geometry::GeometrySlotState::PendingCreate);
    m_pending_create.emplace(id, slot);

    return slot;
}

template <std::derived_from<geometry::Geometry> GeometryT>
P<geometry::GeometrySlotT<GeometryT>> GeometryCollection::find(IndexT id) noexcept
{
    return std::dynamic_pointer_cast<geometry::GeometrySlotT<GeometryT>>(find(id));
}

template <std::derived_from<geometry::Geometry> GeometryT>
P<const geometry::GeometrySlotT<GeometryT>> GeometryCollection::find(IndexT id) const noexcept
{
    return std::dynamic_pointer_cast<geometry::GeometrySlotT<GeometryT>>(find(id));
}
}  // namespace uipc::geometry
