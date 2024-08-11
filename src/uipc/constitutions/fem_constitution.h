#pragma once
#include <uipc/world/constitution.h>
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::constitution
{
class UIPC_CORE_API FEMConstitution : public world::IConstitution
{
    using Base = world::IConstitution;

  public:
    virtual world::ConstitutionType get_type() const noexcept final override;

  protected:
    void apply_to(geometry::SimplicialComplex& sc, Float mass_density) const;
};
}  // namespace uipc::constitution
