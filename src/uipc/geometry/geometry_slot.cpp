#include <uipc./geometry/geometry_slot.h>

namespace uipc::geometry
{
IndexT IGeometrySlot::id() const noexcept
{
    return get_id();
}
Geometry& IGeometrySlot::geometry() noexcept
{
    return get_geometry();
}
const Geometry& IGeometrySlot::geometry() const noexcept
{
    return get_geometry();
}
}  // namespace uipc::geometry
