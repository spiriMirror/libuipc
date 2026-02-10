#include <uipc/constitution/external_articulation_constraint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>

namespace uipc::constitution
{
constexpr U64 ExternalArticulationConstitutionUID = 23;
constexpr U64 ExternalArticulationConstraintUID   = 24;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ExternalArticulationConstraintUID,
                           .name = "ExternalArticulationConstraint",
                           .type = string{builtin::Constraint}});

    uids.push_back(UIDInfo{.uid  = ExternalArticulationConstitutionUID,
                           .name = "ExternalArticulationConstitution",
                           .type = string{builtin::InterAffineBody}});
    return uids;
};

Json ExternalArticulationConstraint::default_config()
{
    return Json::object();
}

ExternalArticulationConstraint::ExternalArticulationConstraint(const Json& config) noexcept
{
    m_config = config;
}

ExternalArticulationConstraint::~ExternalArticulationConstraint() = default;

geometry::Geometry ExternalArticulationConstraint::create_geometry(
    span<S<const geometry::GeometrySlot>> joint_geo) const
{
    vector<IndexT> indices(joint_geo.size(), 0);
    return create_geometry(joint_geo, indices);
}

geometry::Geometry ExternalArticulationConstraint::create_geometry(
    span<S<const geometry::GeometrySlot>> joint_geos, span<IndexT> indices) const
{
    geometry::Geometry R;

    // This is a dummy constitution UID
    // Only used to support the association between Constraint and Constitution
    auto cuid           = R.meta().create<U64>(builtin::constitution_uid, 0);
    view(*cuid).front() = ExternalArticulationConstitutionUID;

    Base::apply_to(R);  // Fill Constraint UID

    UIPC_ASSERT(joint_geos.size() == indices.size(),
                "Selected joints size must match joint geometry size.");

    auto n_joints = static_cast<IndexT>(joint_geos.size());

    // 1. Create joint attribute collection
    auto joint_collection = R["joint"];
    joint_collection->resize(n_joints);

    // 1.1 Create geo_id attribute, indicating which joint geometry we need to refer to
    auto geo_id      = joint_collection->create<IndexT>("geo_id", -1);
    auto geo_id_view = view(*geo_id);
    std::ranges::transform(joint_geos,
                           geo_id_view.begin(),
                           [](const S<const geometry::GeometrySlot>& slot) -> IndexT
                           { return slot->id(); });
    // 1.2 Create index attribute, indicating which joint in the joint geometry we are referring to
    auto index_attr = joint_collection->create<IndexT>("index", -1);
    auto index_view = view(*index_attr);
    std::ranges::copy(indices, index_view.begin());

    // 1.3 Create delta_theta_tilde and delta_theta attributes
    // delta_theta_tilde is the predicted change in joint position
    // delta_theta is the computed change in joint position after simulation step
    joint_collection->create<Float>("delta_theta_tilde", 0.0f);
    joint_collection->create<Float>("delta_theta", 0.0f);

    // 2. Create joint_joint attribute collection
    auto joint_joint_collection = R["joint_joint"];
    joint_joint_collection->resize(n_joints * n_joints);

    // 2.1 Create mass attribute (m_ij) for each joint pair
    joint_joint_collection->create<Float>("mass", 0.0f);

    return R;
}

U64 ExternalArticulationConstraint::get_uid() const noexcept
{
    return ExternalArticulationConstraintUID;
}
}  // namespace uipc::constitution