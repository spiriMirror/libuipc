#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CORE_API FiniteElementConstitution : public IConstitution
{
    using Base = IConstitution;

  protected:
    void apply_to(geometry::SimplicialComplex& sc, Float mass_density, Float thickness = 0.0) const;

  private:
    virtual ConstitutionType get_type() const noexcept final override;
};
}  // namespace uipc::constitution
