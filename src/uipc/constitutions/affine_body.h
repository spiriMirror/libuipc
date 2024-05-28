#pragma once
#include <uipc/world/constitution.h>
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::constitution
{
class AffineBodyConstitution;

class AffineBodyMaterial
{
  public:
    void apply_to(geometry::SimplicialComplex& sc) const;

    AffineBodyMaterial(const AffineBodyMaterial&)            = default;
    AffineBodyMaterial(AffineBodyMaterial&&)                 = default;
    AffineBodyMaterial& operator=(const AffineBodyMaterial&) = default;
    AffineBodyMaterial& operator=(AffineBodyMaterial&&)      = default;

  private:
    friend class AffineBodyConstitution;
    Float m_kappa;
    AffineBodyMaterial(const AffineBodyConstitution&, Float kappa) noexcept;
    const AffineBodyConstitution& m_constitution;
};

class AffineBodyConstitution : public world::IConstitution
{
    using Base = world::IConstitution;

  public:
    AffineBodyConstitution() noexcept;
    AffineBodyMaterial create_material(Float kappa) const;

  public:
    virtual U64 get_uid() const override;

    void apply_to(geometry::SimplicialComplex& sc, Float kappa) const;
};
}  // namespace uipc::constitution
