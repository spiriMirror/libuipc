#pragma once
#include <uipc/common/type_define.h>
#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
class IGeometrySlot
{
  public:
    virtual ~IGeometrySlot() = default;
    IndexT          id() const noexcept;
    Geometry&       geometry() noexcept;
    const Geometry& geometry() const noexcept;

  protected:
    virtual IndexT          get_id() const noexcept       = 0;
    virtual Geometry&       get_geometry() noexcept       = 0;
    virtual const Geometry& get_geometry() const noexcept = 0;
};

template <std::derived_from<Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
class GeometrySlot;
}  // namespace uipc::geometry
