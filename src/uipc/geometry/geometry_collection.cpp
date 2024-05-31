#include <uipc/geometry/geometry_collection.h>

namespace uipc::geometry
{
SizeT IGeometryCollection::size() const noexcept
{
    return get_size();
}

void IGeometryCollection::clear() noexcept
{
    do_clear();
}

void IGeometryCollection::reserve(SizeT size) noexcept
{
    do_reserve(size);
}

void GeometryCollection::destroy(IndexT id) noexcept
{
    m_geometries.erase(id);
}

void GeometryCollection::do_reserve(SizeT size) noexcept
{
    m_geometries.reserve(size);
}

void GeometryCollection::do_clear() noexcept
{
    m_geometries.clear();
}

SizeT GeometryCollection::get_size() const noexcept
{
    return m_geometries.size();
}
}  // namespace uipc::geometry
