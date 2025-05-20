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

S<GeometrySlot> GeometrySlotT<ImplicitGeometry>::do_clone() const
{
    return uipc::make_shared<GeometrySlotT<ImplicitGeometry>>(id(), m_geometry);
}

template class UIPC_CORE_API GeometrySlotT<ImplicitGeometry>;
}  // namespace uipc::geometry

namespace std
{
template class UIPC_CORE_API std::shared_ptr<uipc::geometry::ImplicitGeometrySlot>;
}
