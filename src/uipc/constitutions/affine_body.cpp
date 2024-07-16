#include <uipc/constitutions/affine_body.h>
#include <uipc/builtin/constitution_uid_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>

namespace uipc::constitution
{
void AffineBodyMaterial::apply_to(geometry::SimplicialComplex& sc) const
{
    m_constitution.apply_to(sc, m_kappa, m_mass_density);
}

AffineBodyMaterial::AffineBodyMaterial(const AffineBodyConstitution& ab,
                                       Float                         kappa,
                                       Float mass_density) noexcept
    : m_constitution(ab)
    , m_kappa(kappa)
    , m_mass_density(mass_density)
{
}

AffineBodyConstitution::AffineBodyConstitution() noexcept {}

AffineBodyMaterial AffineBodyConstitution::create_material(Float kappa) const noexcept
{
    return AffineBodyMaterial{*this, kappa};
}

U64 AffineBodyConstitution::get_uid() const noexcept
{
    return 2;
}

std::string_view AffineBodyConstitution::get_name() const noexcept
{
    return builtin::ConstitutionUIDRegister::instance().find(get_uid()).name;
}

world::ConstitutionTypes AffineBodyConstitution::get_type() const noexcept
{
    return world::ConstitutionTypes::AffineBody;
}

P<geometry::AttributeSlot<Float>> AffineBodyConstitution::create_or_get(geometry::SimplicialComplex& sc) const
{
    auto P = sc.instances().find<Float>("kappa");
    if(!P)
        P = sc.instances().create<Float>("kappa");
    return P;
}
void AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density) const
{
    Base::apply_to(sc);

    auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.instances().create<IndexT>(builtin::is_fixed, 0);

    auto kappa_attr = create_or_get(sc);
    auto kappa_view = geometry::view(*kappa_attr);
    std::ranges::fill(kappa_view, kappa);

    auto mass_density_attr = sc.instances().find<Float>(builtin::mass_density);
    if(!mass_density_attr)
        mass_density_attr =
            sc.instances().create<Float>(builtin::mass_density, mass_density);
    auto mass_density_view = geometry::view(*mass_density_attr);
    std::ranges::fill(mass_density_view, mass_density);

    geometry::compute_vertex_mass(sc, mass_density);
}
}  // namespace uipc::constitution
