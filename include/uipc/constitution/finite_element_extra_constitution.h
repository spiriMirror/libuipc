#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API FiniteElementExtraConstitution : public IConstitution
{
    using Base = IConstitution;

  public:
    using Base::Base;

  protected:
    void apply_to(geometry::SimplicialComplex& sc) const;
    // let the derived class to give the uid
    virtual U64 get_uid() const noexcept = 0;
};
}  // namespace uipc::constitution
