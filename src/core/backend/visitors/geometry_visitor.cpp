#include <uipc/backend/visitors/geometry_visitor.h>
#include <uipc/geometry/geometry.h>

namespace uipc::backend
{
GeometryVisitor::GeometryVisitor(geometry::IGeometry& geometry)
    : m_geometry(geometry)
{
}
void GeometryVisitor::collect_attribute_collections(vector<std::string>& names,
                                                    vector<geometry::AttributeCollection*>& collections)
{
    m_geometry.collect_attribute_collections(names, collections);
}
}  // namespace uipc::backend
