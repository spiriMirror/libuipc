#pragma once
#include <uipc/constitutions/fem_constitution.h>

namespace uipc::constitution
{
class UIPC_CORE_API ARAP : public FEMConstitution
{
    using Base = FEMConstitution;

  public:
    ARAP(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density = 1e3) const;

    static Json default_config() noexcept;

  protected:
    virtual U64              get_uid() const noexcept override;
    virtual std::string_view get_name() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
