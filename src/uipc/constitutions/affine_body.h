#pragma once
#include <uipc/world/constitution.h>
#include <uipc/geometry/simplicial_complex.h>
#include <random>

namespace uipc::constitution
{
class AffineBodyConstitution;

class UIPC_CORE_API AffineBodyMaterial
{
  public:
    P<geometry::AttributeSlot<Float>> apply_to(geometry::SimplicialComplex& sc) const;

  private:
    friend class AffineBodyConstitution;
    AffineBodyMaterial(const AffineBodyConstitution&, Float kappa) noexcept;

    const AffineBodyConstitution& m_constitution;
    Float                         m_kappa;
};

class UIPC_CORE_API AffineBodyConstitution : public world::IConstitution
{
    using Base = world::IConstitution;

  public:
    AffineBodyConstitution() noexcept;
    AffineBodyMaterial create_material(Float kappa) const noexcept;

    P<geometry::AttributeSlot<Float>> apply_to(geometry::SimplicialComplex& sc,
                                               Float kappa) const;

  protected:
    virtual U64              get_uid() const noexcept override;
    virtual std::string_view get_name() const noexcept override;
    P<geometry::AttributeSlot<Float>> create_or_get(geometry::SimplicialComplex& sc) const;
};
}  // namespace uipc::constitution
