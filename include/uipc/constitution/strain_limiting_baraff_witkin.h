#pragma once
#include <uipc/constitution/finite_element_constitution.h>
#include <uipc/constitution/elastic_moduli.h>
#include <uipc/common/unit.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API StrainLimitingBaraffWitkinShell : public FiniteElementConstitution
{
    using Base = FiniteElementConstitution;

  public:
    StrainLimitingBaraffWitkinShell(const Json& config = default_config()) noexcept;

    // Separate stretch / shear moduli (cloth: the two deformation modes have
    // independent material parameters, cf. mas-pncg ClothMaterialConfig).
    void apply_to(geometry::SimplicialComplex& sc,
                  const ElasticModuli2D& stretch_moduli,
                  const ElasticModuli2D& shear_moduli,
                  Float mass_density = 2e2,
                  Float thickness    = 0.001_m,
                  Float strain_rate  = 100.0) const;

    // Convenience overload: stretch and shear share one moduli pair.
    void apply_to(geometry::SimplicialComplex& sc,
                  const ElasticModuli2D& moduli = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
                  Float mass_density = 2e2,
                  Float thickness    = 0.001_m,
                  Float strain_rate  = 100.0) const
    {
        apply_to(sc, moduli, moduli, mass_density, thickness, strain_rate);
    }

    static Json default_config() noexcept;

  protected:
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
