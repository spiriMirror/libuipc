#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API FiniteElementConstitution : public IConstitution
{
    using Base = IConstitution;

  protected:
    void apply_to(geometry::SimplicialComplex& sc, Float mass_density, Float thickness = 0.0) const;
};
}  // namespace uipc::constitution
