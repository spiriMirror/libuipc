#pragma once
#include <uipc/geometry/geometry.h>

namespace uipc::geometry
{
class ImplicitGeometry : public Geometry
{
  public:
    ImplicitGeometry();

  protected:
    // Inherited via Geometry
    std::string_view get_type() const noexcept override;
};
}  // namespace uipc::geometry
