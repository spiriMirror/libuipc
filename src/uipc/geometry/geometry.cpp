#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
IGeometry::IGeometry(const std::string& type)
    : m_type(type)
{
}
std::string_view IGeometry::type() const
{
    return m_type;
}
}  // namespace uipc::geometry