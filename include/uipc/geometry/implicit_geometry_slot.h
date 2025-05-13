#pragma once
#include <uipc/geometry/geometry_slot.h>
#include <uipc/geometry/implicit_geometry.h>

namespace uipc::geometry
{
template <>
class UIPC_CORE_API GeometrySlotT<ImplicitGeometry> : public GeometrySlot
{
  public:
    GeometrySlotT(IndexT id, const ImplicitGeometry& geometry);

    ImplicitGeometry&       geometry() noexcept;
    const ImplicitGeometry& geometry() const noexcept;

  protected:
    Geometry&       get_geometry() noexcept override;
    const Geometry& get_geometry() const noexcept override;
    S<GeometrySlot> do_clone() const override;

  private:
    ImplicitGeometry m_geometry;
};

using ImplicitGeometrySlot = GeometrySlotT<ImplicitGeometry>;
}  // namespace uipc::geometry

namespace std
{
extern template class std::shared_ptr<uipc::geometry::ImplicitGeometrySlot>;
}
