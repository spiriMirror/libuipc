#include <uipc/geometry/implicit_geometry_slot.h>

namespace uipc::geometry
{
GeometrySlotT<ImplicitGeometry>::GeometrySlotT(IndexT id, const ImplicitGeometry& geometry)
    : GeometrySlot{id}
    , m_geometry{geometry}
{
}

ImplicitGeometry& GeometrySlotT<ImplicitGeometry>::geometry() noexcept
{
    return m_geometry;
}

const ImplicitGeometry& GeometrySlotT<ImplicitGeometry>::geometry() const noexcept
{
    return m_geometry;
}

Geometry& GeometrySlotT<ImplicitGeometry>::get_geometry() noexcept
{
    return m_geometry;
}

const Geometry& GeometrySlotT<ImplicitGeometry>::get_geometry() const noexcept
{
    return m_geometry;
}
}  // namespace uipc::geometry
