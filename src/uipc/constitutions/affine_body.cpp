#include <uipc/constitutions/affine_body.h>

namespace uipc::constitution
{
void AffineBodyMaterial::apply_to(geometry::SimplicialComplex& sc) const
{
    m_constitution.apply_to(sc, m_kappa);
}

AffineBodyMaterial::AffineBodyMaterial(const AffineBodyConstitution& ab, Float kappa) noexcept
    : m_constitution(ab)
    , m_kappa(kappa)
{
}

AffineBodyConstitution::AffineBodyConstitution() noexcept {}

AffineBodyMaterial AffineBodyConstitution::create_material(Float kappa) const
{
    return AffineBodyMaterial(*this, kappa);
}

U64 AffineBodyConstitution::get_uid() const
{
    return 1;
}

void AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc, Float kappa) const
{
    Base::apply_to(sc);
    auto P = sc.instances().find<Float>("abd::kappa");
    if(!P)
        P = sc.instances().create<Float>("abd::kappa", kappa);
    else
    {
        auto v = geometry::view(*P);
        std::ranges::fill(v, kappa);
    }
}
}  // namespace uipc::constitution
