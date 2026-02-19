#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API InterAffineBodyExtraConstitution : public IConstitution
{
    using Base = IConstitution;

  public:
    using Base::Base;

  protected:
    void apply_to(geometry::SimplicialComplex& sc) const;
    virtual U64 get_uid() const noexcept = 0;
};
}  // namespace uipc::constitution
