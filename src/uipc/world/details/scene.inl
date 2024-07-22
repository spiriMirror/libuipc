namespace uipc::world
{
template <std::derived_from<geometry::Geometry> GeometryT>
ObjectGeometrySlots<GeometryT> world::Scene::Geometries::find(IndexT id) && noexcept
{
    return {m_scene.m_impl.geometries.find(id),
            m_scene.m_impl.rest_geometries.find(id)};
}

template <std::derived_from<geometry::Geometry> GeometryT>
ObjectGeometrySlots<const GeometryT> Scene::CGeometries::find(IndexT id) && noexcept
{
    return {m_scene.m_impl.geometries.find(id),
            m_scene.m_impl.rest_geometries.find(id)};
}
}  // namespace uipc::world
