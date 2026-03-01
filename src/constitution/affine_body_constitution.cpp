#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/geometry/utils/compute_mesh_volume.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    // create 8 AffineBody constitution uids
    auto affine_body = string{builtin::AffineBody};
    uids.push_back(UIDInfo{.uid = 1, .name = "OrthoPotential", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 2, .name = "ARAP", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 3, .name = "AffineBody", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 4, .name = "AffineBody", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 5, .name = "AffineBody", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 6, .name = "AffineBody", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 7, .name = "AffineBody", .type = affine_body});
    uids.push_back(UIDInfo{.uid = 8, .name = "AffineBody", .type = affine_body});
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
    : m_config(config)
{
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

void AffineBodyConstitution::setup_abd_attributes(geometry::SimplicialComplex& sc,
                                                   Float kappa,
                                                   Float mass_density,
                                                   Float volume) const
{
    auto cuid = sc.meta().find<U64>(builtin::constitution_uid);
    if(!cuid)
        cuid = sc.meta().create<U64>(builtin::constitution_uid, 0);
    geometry::view(*cuid).front() = uid();

    // affine body objects' transform changing over time
    // label transform as evolving for streaming optimization
    sc.transforms().is_evolving(true);

    auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
    if(!dof_offset)
        dof_offset = sc.meta().create<IndexT>(builtin::dof_offset, -1);

    auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
    if(!dof_count)
        dof_count = sc.meta().create<IndexT>(builtin::dof_count, 0);

    auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.instances().create<IndexT>(builtin::is_fixed, 0);

    auto is_dynamic = sc.instances().find<IndexT>(builtin::is_dynamic);
    if(!is_dynamic)
        is_dynamic = sc.instances().create<IndexT>(builtin::is_dynamic, 1);

    auto external_kinetic = sc.instances().find<IndexT>(builtin::external_kinetic);
    if(!external_kinetic)
        external_kinetic = sc.instances().create<IndexT>(builtin::external_kinetic, 0);

    auto velocity = sc.instances().find<Matrix4x4>(builtin::velocity);
    if(!velocity)
        velocity = sc.instances().create<Matrix4x4>(builtin::velocity, Matrix4x4::Zero());

    // affine body always turns off self-collision by default
    auto self_collision = sc.meta().find<IndexT>(builtin::self_collision);
    if(!self_collision)
        self_collision = sc.meta().create<IndexT>(builtin::self_collision, 0);

    auto kappa_attr = sc.instances().find<Float>("kappa");
    if(!kappa_attr)
        kappa_attr = sc.instances().create<Float>("kappa", kappa);
    auto kappa_view = geometry::view(*kappa_attr);
    std::ranges::fill(kappa_view, kappa);

    if constexpr(uipc::RUNTIME_CHECK)
    {
        UIPC_ASSERT(volume > 0, "Volume of the mesh is non-positive ({}), which is not allowed.", volume);
    }

    auto meta_volume = sc.meta().find<Float>(builtin::volume);
    if(!meta_volume)
        meta_volume = sc.meta().create<Float>(builtin::volume, volume);
    else
        geometry::view(*meta_volume).front() = volume;

    auto meta_mass = sc.meta().find<Float>(builtin::mass_density);
    if(!meta_mass)
        meta_mass = sc.meta().create<Float>(builtin::mass_density, mass_density);
    else
        geometry::view(*meta_mass).front() = mass_density;
}

void AffineBodyConstitution::apply_to(geometry::SimplicialComplex& sc, Float kappa, Float mass_density) const
{
    auto volume = geometry::compute_mesh_volume(sc);
    setup_abd_attributes(sc, kappa, mass_density, volume);
}

Json AffineBodyConstitution::default_config() noexcept
{
    Json j    = Json::object();
    j["name"] = "OrthoPotential";
    return j;
}
}  // namespace uipc::constitution
