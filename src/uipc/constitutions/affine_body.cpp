#include <uipc/constitutions/affine_body.h>
#include <uipc/builtin/constitution_uid_register.h>
#include <uipc/builtin/attribute_name.h>
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

P<geometry::AttributeSlot<Float>> AffineBodyConstitution::create_or_get(geometry::SimplicialComplex& sc) const
{
    auto P = sc.instances().find<Float>("abd::kappa");
    if(!P)
        P = sc.instances().create<Float>("abd::kappa");
    return P;
}

P<geometry::AttributeSlot<Float>> AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc,
                                                                   Float kappa) const
{
    Base::apply_to(sc);

    auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.instances().create<IndexT>(builtin::is_fixed, 0);

    auto P = create_or_get(sc);
    auto v = geometry::view(*P);
    std::ranges::fill(v, kappa);
    return P;
}
}  // namespace uipc::constitution
