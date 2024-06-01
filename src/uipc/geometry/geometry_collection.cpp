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

IndexT IGeometryCollection::next_id() const noexcept
{
    return get_next_id();
}

void GeometryCollection::destroy(IndexT id) noexcept
{
    auto it = m_geometries.find(id);
    if(it != m_geometries.end())
        m_geometries.erase(it);
    else
        UIPC_WARN_WITH_LOCATION("Trying to destroy a non-existing Geometry Slot ({}), ignored.",
                                id);
}

void GeometryCollection::pending_destroy(IndexT id) noexcept
{
    if(m_geometries.find(id) != m_geometries.end())  // if it exists
        m_pending_destroy.insert(id);                // mark it to be destroyed
    else if(m_pending_create.find(id) != m_pending_create.end())  // if it is pending to be created
        m_pending_create.erase(id);  // cancel the creation
    else
        UIPC_WARN_WITH_LOCATION("Trying to destroy a non-existing Geometry Slot ({}), ignored.",
                                id);
}

void GeometryCollection::solve_pending() noexcept
{
    for(auto& [id, geo] : m_pending_create)
    {
        UIPC_ASSERT(m_geometries.find(id) == m_geometries.end(),
                    "GeometrySlot ({}) already exists. Why can this happen?",
                    id);
        m_geometries.emplace(id, geo);
    }
    m_pending_create.clear();

    for(auto id : m_pending_destroy)
    {
        UIPC_ASSERT(m_geometries.find(id) != m_geometries.end(),
                    "GeometrySlot ({}) does not exist. Why can this happen?",
                    id);
        m_geometries.erase(id);
    }
    m_pending_destroy.clear();
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
IndexT GeometryCollection::get_next_id() const noexcept
{
    return m_next_id;
}
}  // namespace uipc::geometry
