#pragma once
#include <uipc/world/constitution.h>
#include <uipc/geometry/simplicial_complex.h>
#include <random>
#include <uipc/common/unit.h>

namespace uipc::constitution
{
class AffineBodyConstitution;

class UIPC_CORE_API AffineBodyMaterial
{
  public:
    void apply_to(geometry::SimplicialComplex& sc) const;

  private:
    friend class AffineBodyConstitution;
    AffineBodyMaterial(const AffineBodyConstitution&, Float kappa, Float mass_density = 1e3) noexcept;

    const AffineBodyConstitution& m_constitution;
    Float                         m_kappa;
    Float                         m_mass_density;
};

class UIPC_CORE_API AffineBodyConstitution : public world::IConstitution
{
    using Base = world::IConstitution;

  public:
    AffineBodyConstitution(const Json& config = default_config()) noexcept;
    AffineBodyMaterial create_material(Float kappa) const noexcept;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density = 1e3) const;

    static Json default_config() noexcept;

  protected:
    virtual U64                     get_uid() const noexcept override;
    virtual std::string_view        get_name() const noexcept override;
    virtual world::ConstitutionType get_type() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
