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

    // Raw kappa in the Discrete Shells edge energy. The historical default
    // retains its numeric value; it is not a per-area coefficient.
    void apply_to(geometry::SimplicialComplex& sc, Float bending_stiffness = 100.0_kPa);

    // Formula-based overload: D = E·h³/(12·(1-ν²)), where the vertex
    // `thickness` attribute stores the one-sided offset r and h = 2r is the
    // full material thickness. The radius is averaged over each edge. A
    // membrane constitution sets it and must be applied first.
    void apply_to(geometry::SimplicialComplex& sc, Float young_modulus, Float poisson_ratio);

    // Compute D from a one-sided thickness radius r (full thickness h = 2r).
    static Float bending_stiffness(Float young_modulus, Float poisson_ratio, Float thickness) noexcept;

    static Json default_config();


  private:
    virtual U64 get_uid() const noexcept final override;
    Json        m_config;
};
}  // namespace uipc::constitution
