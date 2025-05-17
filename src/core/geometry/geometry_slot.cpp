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

S<GeometrySlot> GeometrySlot::clone() const
{
    return do_clone();
}

void GeometrySlot::id(IndexT id) noexcept
{
    m_id = id;
}

void GeometrySlot::state(GeometrySlotState state) noexcept
{
    m_state = state;
}

GeometrySlotT<Geometry>::GeometrySlotT(IndexT id, const Geometry& geometry)
    : GeometrySlot{id}
    , m_geometry{geometry}
{
}

Geometry& GeometrySlotT<Geometry>::get_geometry() noexcept
{
    return m_geometry;
}

const Geometry& GeometrySlotT<Geometry>::get_geometry() const noexcept
{
    return m_geometry;
}

S<GeometrySlot> GeometrySlotT<Geometry>::do_clone() const
{
    return uipc::make_shared<GeometrySlotT<Geometry>>(id(), m_geometry);
}

template class UIPC_CORE_API GeometrySlotT<Geometry>;
}  // namespace uipc::geometry

namespace std
{
template class UIPC_CORE_API std::shared_ptr<uipc::geometry::GeometrySlotT<uipc::geometry::Geometry>>;
template class UIPC_CORE_API std::shared_ptr<uipc::geometry::GeometrySlot>;
}  // namespace std
