#include <uipc/geometry/simplicial_complex.h>

namespace uipc::world
{
template <std::derived_from<geometry::Geometry> GeometryT>
auto ObjectGeometries<GeometryT>::geometries() noexcept -> Geometries
{
    return Geometries{m_geometries};
}

template <std::derived_from<geometry::Geometry> GeometryT>
auto ObjectGeometries<GeometryT>::rest_geometries() noexcept -> Geometries
{
    return Geometries{m_rest_geometries};
}

template <std::derived_from<geometry::Geometry> GeometryT>
ObjectGeometries<GeometryT>::ObjectGeometries(AutoCollectionT& geometries,
                                              AutoCollectionT& rest_geometries) noexcept
    : m_geometries(geometries)
    , m_rest_geometries(rest_geometries)
{
}

template <std::derived_from<geometry::Geometry> GeometryT>
auto ObjectGeometries<GeometryT>::Geometries::view() && noexcept -> span<AutoGeometryT>
{
    return m_geometries.view();
}

template <std::derived_from<geometry::Geometry> GeometryT>
ObjectGeometries<GeometryT>::Geometries::Geometries(AutoCollectionT& geometries) noexcept
    : m_geometries(geometries)
{
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
U64 Object::find_or_create() const
{
    auto index     = m_uid_to_index.size();
    auto hash_code = typeid(GeometryT).hash_code();
    auto it        = m_uid_to_index.find(hash_code);
    if(it != m_uid_to_index.end())
    {
        return it->second;
    }
    else
    {
        m_uid_to_index[hash_code] = index;
        m_rest_geometry.push_back(std::make_unique<GeometryCollection<GeometryT>>());
        m_geometry.push_back(std::make_unique<GeometryCollection<GeometryT>>());
        return index;
    }
}

using GeometryT = geometry::SimplicialComplex;
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometries<GeometryT> Object::find()
{
    using ObjectGeometriesT   = ObjectGeometries<GeometryT>;
    using GeometryCollectionT = typename ObjectGeometriesT::CollectionT;

    auto index = find_or_create<GeometryT>();
    return ObjectGeometriesT{static_cast<GeometryCollectionT&>(*m_geometry[index]),
                             static_cast<GeometryCollectionT&>(*m_rest_geometry[index])};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
auto Object::find() const -> ObjectGeometries<const GeometryT>
{
    using ObjectGeometriesT   = ObjectGeometries<const GeometryT>;
    using GeometryCollectionT = typename ObjectGeometriesT::CollectionT;

    auto index = find_or_create<GeometryT>();
    return ObjectGeometriesT{
        static_cast<const GeometryCollectionT&>(*m_geometry[index]),
        static_cast<const GeometryCollectionT&>(*m_rest_geometry[index])};
}

using GeometryT = geometry::SimplicialComplex;
template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometries<GeometryT> Object::push_back(const GeometryT& geometry)
{
    using ObjectGeometriesT = ObjectGeometries<GeometryT>;
    auto  index             = find_or_create<GeometryT>();
    auto& typed_geo_collection =
        static_cast<GeometryCollection<GeometryT>&>(*m_geometry[index]);
    auto& typed_rest_geo_collection =
        static_cast<GeometryCollection<GeometryT>&>(*m_rest_geometry[index]);
    typed_geo_collection.push_back(geometry);
    typed_rest_geo_collection.push_back(geometry);
    return ObjectGeometriesT{typed_geo_collection, typed_rest_geo_collection};
}

template <std::derived_from<geometry::Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
ObjectGeometries<GeometryT> Object::push_back(const GeometryT& rest_geometry,
                                              const GeometryT& geometry)
{
    using ObjectGeometriesT = ObjectGeometries<GeometryT>;

    auto  index = find_or_create<GeometryT>();
    auto& typed_geo_collection =
        static_cast<GeometryCollection<GeometryT>&>(*m_geometry[index]);
    auto& typed_rest_geo_collection =
        static_cast<GeometryCollection<GeometryT>&>(*m_rest_geometry[index]);
    typed_geo_collection.push_back(geometry);
    typed_rest_geo_collection.push_back(rest_geometry);

    return ObjectGeometriesT{typed_geo_collection, typed_rest_geo_collection};
}
}  // namespace uipc::world
