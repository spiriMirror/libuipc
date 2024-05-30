#include <uipc/constitutions/affine_body.h>
#include <uipc/builtin/constitution_uid_register.h>

namespace uipc::constitution
{
P<geometry::AttributeSlot<Float>> AffineBodyMaterial::apply_to(geometry::SimplicialComplex& sc) const
{
    return m_constitution.apply_to(sc, m_kappa);
}

AffineBodyMaterial::AffineBodyMaterial(const AffineBodyConstitution& ab, Float kappa) noexcept
    : m_constitution(ab)
    , m_kappa(kappa)
{
}

AffineBodyConstitution::AffineBodyConstitution() noexcept {}

AffineBodyMaterial AffineBodyConstitution::create_material(Float kappa) const noexcept
{
    return AffineBodyMaterial{*this, kappa};
}

U64 AffineBodyConstitution::get_uid() const noexcept
{
    return 1;
}

std::string_view AffineBodyConstitution::get_name() const noexcept
{
    return builtin::ConstitutionUIDRegister::instance().find(get_uid()).name;
}

P<geometry::AttributeSlot<Float>> AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc,
                                                                   Float kappa) const
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
    return P;
}
}  // namespace uipc::constitution
