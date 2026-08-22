#pragma once
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/common/unit.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API DiscreteShellBending : public FiniteElementExtraConstitution
{
    using Base = FiniteElementExtraConstitution;

  public:
    DiscreteShellBending(const Json& json = default_config());

    void apply_to(geometry::SimplicialComplex& sc, Float bending_stiffness = 100.0_kPa);

    // Formula-based overload: κ = E·t³/(12·(1-ν²)) (the classical shell
    // bending stiffness), used literally as the effective stiffness since
    // the backend bending measure is the element area. The thickness t is
    // read per edge from the mesh's vertex "thickness" attribute (average of
    // the two endpoints) — a membrane constitution (stretch) sets it, and
    // must be applied first.
    void apply_to(geometry::SimplicialComplex& sc, Float young_modulus, Float poisson_ratio);

    // The raw attribute value for a given material: E·t³/(12·(1-ν²)).
    static Float bending_stiffness(Float young_modulus, Float poisson_ratio, Float thickness) noexcept;

    static Json default_config();


  private:
    virtual U64 get_uid() const noexcept final override;
    Json        m_config;
};
}  // namespace uipc::constitution
