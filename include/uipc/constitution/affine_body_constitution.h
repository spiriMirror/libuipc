#pragma once
#include <uipc/constitution/constitution.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::constitution
{
class AffineBodyConstitution;

class UIPC_CONSTITUTION_API AffineBodyMaterial
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

class UIPC_CONSTITUTION_API AffineBodyConstitution : public IConstitution
{
    using Base = IConstitution;

  public:
    AffineBodyConstitution(const Json& config = default_config()) noexcept;
    AffineBodyMaterial create_material(Float kappa) const noexcept;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density = 1e3) const;

    static Json default_config() noexcept;

  protected:
    virtual U64              get_uid() const noexcept override;

  private:
    Json m_config;
};
}  // namespace uipc::constitution
