namespace uipc::core
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<GeometryT> Object::Geometries::create(const GeometryT& geometry,
                                                          const GeometryT& rest_geometry) &&
{
    m_object.m_geometry_ids.push_back(m_object.geometry_collection().next_id());

    UIPC_ASSERT(m_object.geometry_collection().next_id()
                    == m_object.rest_geometry_collection().next_id(),
                "Geometry sim_systems element count ({}) is not equal to rest geometry sim_systems element count ({}), why?",
                m_object.geometry_collection().size(),
                m_object.rest_geometry_collection().size());

    if(m_object.scene_started() || m_object.scene_pending())
    {
        return {m_object.geometry_collection().pending_emplace(geometry),
                m_object.rest_geometry_collection().pending_emplace(rest_geometry)};
    }
    else // before `world.init(scene)` is called
    {
        return {m_object.geometry_collection().emplace(geometry),
                m_object.rest_geometry_collection().emplace(rest_geometry)};
    }
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<GeometryT> Object::Geometries::create(const GeometryT& geometry) &&
{
    return std::move(*this).template create<GeometryT>(geometry, geometry);
}
}  // namespace uipc::core
