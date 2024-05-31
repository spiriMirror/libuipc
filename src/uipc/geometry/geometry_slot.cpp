#include <uipc./geometry/geometry_slot.h>

namespace uipc::geometry
{
IndexT uipc::geometry::IGeometrySlot::id() const noexcept
{
    return get_id();
}
}  // namespace uipc::geometry
