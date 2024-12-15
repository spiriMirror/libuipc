#pragma once
#include <string>
#include <uipc/geometry/attribute_collection.h>
namespace uipc::geometry
{
class IGeometry;
}
namespace uipc::backend
{
class UIPC_CORE_API GeometryVisitor
{
  public:
    GeometryVisitor(geometry::IGeometry& geometry);
    void collect_attribute_collections(vector<std::string>& names,
                                       vector<geometry::AttributeCollection*>& collections);

  private:
    geometry::IGeometry& m_geometry;
};
}  // namespace uipc::backend
