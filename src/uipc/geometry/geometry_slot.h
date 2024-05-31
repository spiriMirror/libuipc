#pragma once
#include <uipc/common/type_define.h>
#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
class IGeometrySlot
{
  public:
    virtual ~IGeometrySlot() = default;
    IndexT id() const noexcept;

  protected:
    virtual IndexT get_id() const noexcept = 0;
};

template <std::derived_from<Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
class GeometrySlot;
}  // namespace uipc::geometry
