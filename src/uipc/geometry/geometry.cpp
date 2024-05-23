#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
std::string_view IGeometry::type() const
{
    return get_type();
}
}  // namespace uipc::geometry