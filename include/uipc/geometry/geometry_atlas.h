#pragma once
#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
class UIPC_CORE_API GeometryAtlas
{
    class Impl;

  public:
    GeometryAtlas();
    ~GeometryAtlas();

    // Common Geometry

    IndexT          create(const Geometry& geo);
    const Geometry* find(IndexT id) const;

    // Top Most AttributeCollection

    void create(std::string_view name, const AttributeCollection& ac);
    const AttributeCollection* find(std::string_view name) const;
    Json                       to_json() const;
    void                       from_json(const Json& j);

  private:
    U<Impl> m_impl;
};
}  // namespace uipc::geometry
