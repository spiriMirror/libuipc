#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/geometry.h>

namespace uipc::constitution
{
class UIPC_CORE_API Constraint : public IConstitution
{
  public:
    Constraint() noexcept;

  protected:
    virtual U64 get_uid() const noexcept = 0;

    void apply_to(geometry::Geometry& geo) const;
};
}  // namespace uipc::constitution
