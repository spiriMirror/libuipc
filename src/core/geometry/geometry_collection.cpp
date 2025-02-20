#include <uipc/geometry/geometry_collection.h>
#include <uipc/common/enumerate.h>

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
    m_dirty = true;

    auto it = m_geometries.find(id);
    if(it != m_geometries.end())
    {
        m_geometries.erase(it);
    }
    else
        UIPC_WARN_WITH_LOCATION("Trying to destroy a non-existing Geometry Slot ({}), ignored.",
                                id);
}

void GeometryCollection::pending_destroy(IndexT id) noexcept
{
    m_dirty = true;
    if(auto it = m_geometries.find(id); it != m_geometries.end())  // if it exists
    {
        it->second->state(GeometrySlotState::PendingDestroy);
        m_pending_destroy.insert(id);  // mark it to be destroyed
    }
    else if(m_pending_create.find(id) != m_pending_create.end())  // if it is pending to be created
    {
        UIPC_INFO_WITH_LOCATION(
            "Try to destroy a pending create Geometry Slot ({}), so we cancel the creation. "
            "This may be eliminated by optimizing your implementation.",
            id);
        m_pending_create.erase(id);  // cancel the creation
    }
    else
    {
        UIPC_WARN_WITH_LOCATION("Trying to destroy a non-existing Geometry Slot ({}), ignored.",
                                id);
    }
}

void GeometryCollection::solve_pending() noexcept
{
    m_dirty = true;

    // put the pending create into the geometries
    for(auto& [id, geo] : m_pending_create)
    {
        UIPC_ASSERT(m_geometries.find(id) == m_geometries.end(),
                    "GeometrySlot ({}) already exists. Why can this happen?",
                    id);

        UIPC_ASSERT(geo->state() == GeometrySlotState::PendingCreate,
                    "GeometrySlot ({}) is not in PendingCreate state. Why can this happen?",
                    id);

        geo->state(GeometrySlotState::Normal);
        m_geometries.emplace(id, geo);
    }
    m_pending_create.clear();

    for(auto id : m_pending_destroy)
    {
        auto it = m_geometries.find(id);
        UIPC_ASSERT(it != m_geometries.end(),
                    "GeometrySlot ({}) does not exist. Why can this happen?",
                    id);

        UIPC_ASSERT(it->second->state() == GeometrySlotState::PendingDestroy,
                    "GeometrySlot ({}) is not in PendingDestroy state. Why can this happen?",
                    id);
        m_geometries.erase(it);
    }
    m_pending_destroy.clear();
}

span<S<geometry::GeometrySlot>> GeometryCollection::geometry_slots() const noexcept
{
    flush();
    return m_geometry_slots;
}

span<S<geometry::GeometrySlot>> GeometryCollection::pending_create_slots() const noexcept
{
    flush();
    return m_pending_create_slots;
}

span<IndexT> GeometryCollection::pending_destroy_ids() const noexcept
{
    flush();
    return m_pending_destroy_ids;
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
void GeometryCollection::flush() const
{
    if(m_dirty)
    {
        m_geometry_slots.resize(m_geometries.size());
        m_pending_create_slots.resize(m_pending_create.size());
        m_pending_destroy_ids.resize(m_pending_destroy.size());

        for(auto&& [I, geo] : enumerate(m_geometries))
        {
            m_geometry_slots[I] = geo.second;
        }

        std::sort(m_geometry_slots.begin(),
                  m_geometry_slots.end(),
                  [](const S<geometry::GeometrySlot>& a, const S<geometry::GeometrySlot>& b)
                  { return a->id() < b->id(); });

        for(auto&& [I, geo] : enumerate(m_pending_create))
        {
            m_pending_create_slots[I] = geo.second;
        }

        std::sort(m_pending_create_slots.begin(),
                  m_pending_create_slots.end(),
                  [](const S<geometry::GeometrySlot>& a, const S<geometry::GeometrySlot>& b)
                  { return a->id() < b->id(); });

        std::ranges::copy(m_pending_destroy, m_pending_destroy_ids.begin());

        m_dirty = false;
    }
}


S<geometry::GeometrySlot> GeometryCollection::find(IndexT id) noexcept
{
    if(auto it = m_pending_destroy.find(id); it != m_pending_destroy.end())
    {
        return {};
    }

    if(auto it = m_geometries.find(id); it != m_geometries.end())
    {
        return it->second;
    }

    if(auto it = m_pending_create.find(id); it != m_pending_create.end())
    {
        return it->second;
    }

    return {};
}

S<const geometry::GeometrySlot> GeometryCollection::find(IndexT id) const noexcept
{
    return const_cast<GeometryCollection*>(this)->find(id);
}
}  // namespace uipc::geometry
