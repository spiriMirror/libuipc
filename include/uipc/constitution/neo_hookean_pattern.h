#pragma once
#include <uipc/constitution/finite_element_constitution.h>
#include <uipc/constitution/elastic_moduli.h>
#include <uipc/common/unit.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API NeoHookeanPattern : public FiniteElementConstitution
{
    using Base = FiniteElementConstitution;

  public:
    NeoHookeanPattern(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc,
                  const vector<Float>& X_bars,
                  const ElasticModuli& moduli = ElasticModuli::youngs_poisson(10.0_MPa, 0.49),
                  Float mass_density = 1e3,
                  Float thickness    = 0.01_m) const;

    static Json default_config() noexcept;

  protected:
    virtual U64 get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
