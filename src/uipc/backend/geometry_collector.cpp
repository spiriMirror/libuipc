#include <uipc/backend/geometry_collector.h>

namespace uipc::backend
{
GeometryCollector::GeometryCollector(SceneVisitor visitor) noexcept
    : m_visitor(visitor)
{
}
}  // namespace uipc::backend
