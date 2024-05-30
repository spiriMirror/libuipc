namespace uipc::world
{
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
void GeometryCollection<GeometryT>::push_back(const GeometryT& geometry)
{
    m_geometries.push_back(geometry);
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
void GeometryCollection<GeometryT>::push_back(const GeometryT&& geometry)
{
    m_geometries.push_back(std::move(geometry));
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
span<GeometryT> GeometryCollection<GeometryT>::view() noexcept
{
    return m_geometries;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
span<const GeometryT> GeometryCollection<GeometryT>::view() const noexcept
{
    return m_geometries;
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
void uipc::world::GeometryCollection<GeometryT>::do_resize(SizeT size) noexcept
{
    m_geometries.resize(size);
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
void GeometryCollection<GeometryT>::do_reserve(SizeT size) noexcept
{
    m_geometries.reserve(size);
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
void GeometryCollection<GeometryT>::do_clear() noexcept
{
    m_geometries.clear();
}
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_const_v<GeometryT>)
SizeT GeometryCollection<GeometryT>::get_size() const noexcept
{
    return m_geometries.size();
}
}  // namespace uipc::world
