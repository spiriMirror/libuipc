namespace uipc::world
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<GeometryT> Object::Geometries::find(IndexT id) &&
{
    return {m_object.m_geometries.find<GeometryT>(id),
            m_object.m_rest_geometries.find<GeometryT>(id)};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<const GeometryT> Object::CGeometries::find(IndexT id) &&
{
    return {m_object.m_geometries.find<GeometryT>(id),
            m_object.m_rest_geometries.find<GeometryT>(id)};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<GeometryT> Object::Geometries::create(const GeometryT& geometry) &&
{
    return {m_object.m_geometries.emplace(geometry),
            m_object.m_rest_geometries.emplace(geometry)};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometrySlots<GeometryT> Object::Geometries::create(const GeometryT& geometry,
                                                   const GeometryT& rest_geometry) &&
{
    return {m_object.m_geometries.emplace(geometry),
            m_object.m_rest_geometries.emplace(rest_geometry)};
}
}  // namespace uipc::world
