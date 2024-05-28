#pragma once
#include <string_view>
#include <uipc/geometry/geometry.h>
#include <uipc/world/constitution_uid.h>
#include <uipc/builtin/constitution_uid_register.h>
namespace uipc::world
{
class IConstitution
{
  public:
    virtual ~IConstitution() = default;
    U64 uid() const;

  protected:
    void        apply_to(geometry::Geometry& geo) const;
    virtual U64 get_uid() const = 0;
};
}  // namespace uipc::world
