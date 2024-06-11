#include <uipc/geometry/geometry_slot.h>

namespace uipc::geometry
{
GeometrySlot::GeometrySlot(IndexT id) noexcept
    : m_id{id}
{
}

IndexT GeometrySlot::id() const noexcept
{
    return m_id;
}

Geometry& GeometrySlot::geometry() noexcept
{
    return get_geometry();
}

const Geometry& GeometrySlot::geometry() const noexcept
{
    return get_geometry();
}

GeometrySlotState GeometrySlot::state() const noexcept
{
    return m_state;
}

void GeometrySlot::state(GeometrySlotState state) noexcept 
{
    m_state = state;
}
}  // namespace uipc::geometry
