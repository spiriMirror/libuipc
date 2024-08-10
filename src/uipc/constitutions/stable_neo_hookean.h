#pragma once
#include <uipc/world/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class UIPC_CORE_API StableNeoHookeanParms
{
  public:
    static StableNeoHookeanParms EG(Float E, Float G);
    static StableNeoHookeanParms ML(Float mu, Float lambda);
    static StableNeoHookeanParms EP(Float E, Float poisson);

  private:
    friend class StableNeoHookean;
    Float m_mu;
    Float m_lambda;
    Float m_poisson_ratio;
};

class UIPC_CORE_API StableNeoHookean : public world::IConstitution
{
    using Base = world::IConstitution;

  public:
    StableNeoHookean(const Json& config = default_config()) noexcept;

    void apply_to(geometry::SimplicialComplex& sc,
                  const StableNeoHookeanParms& parms,
                  Float                        mass_density = 1e3) const;

    static Json default_config() noexcept;

  protected:
    virtual U64                     get_uid() const noexcept override;
    virtual std::string_view        get_name() const noexcept override;
    virtual world::ConstitutionType get_type() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
