#include <uipc/constitution/affine_body.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    // create 8 AffineBody constitution uids
    uids.push_back(UIDInfo{.uid = 1, .name = "AffineBody::OrthoPotential"});
    uids.push_back(UIDInfo{.uid = 2, .name = "AffineBody::ARAP"});
    uids.push_back(UIDInfo{.uid = 3, .name = "AffineBody"});
    uids.push_back(UIDInfo{.uid = 4, .name = "AffineBody"});
    uids.push_back(UIDInfo{.uid = 5, .name = "AffineBody"});
    uids.push_back(UIDInfo{.uid = 6, .name = "AffineBody"});
    uids.push_back(UIDInfo{.uid = 7, .name = "AffineBody"});
    uids.push_back(UIDInfo{.uid = 8, .name = "AffineBody"});
    return uids;
}

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

AffineBodyConstitution::AffineBodyConstitution(const Json& config) noexcept
{
    m_config = config;
}

AffineBodyMaterial AffineBodyConstitution::create_material(Float kappa) const noexcept
{
    return AffineBodyMaterial{*this, kappa};
}

U64 AffineBodyConstitution::get_uid() const noexcept
{
    if(m_config["name"] == "OrthoPotential")
        return 1;
    else if(m_config["name"] == "ARAP")
        return 2;

    return 1;
}

ConstitutionType AffineBodyConstitution::get_type() const noexcept
{
    return ConstitutionType::AffineBody;
}

void AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density) const
{
    Base::apply_to(sc);

    auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.instances().create<IndexT>(builtin::is_fixed, 0);

    auto kappa_attr = sc.instances().find<Float>("kappa");
    if(!kappa_attr)
        kappa_attr = sc.instances().create<Float>("kappa", kappa);
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
Json AffineBodyConstitution::default_config() noexcept
{
    Json j    = Json::object();
    j["name"] = "OrthoPotential";
    return j;
}
}  // namespace uipc::constitution
