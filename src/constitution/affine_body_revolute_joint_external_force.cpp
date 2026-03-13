#include <uipc/constitution/affine_body_revolute_joint_external_force.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 668;

static constexpr std::string_view ExtForceIsConstrainedName = "external_torque/is_constrained";

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyRevoluteJointExternalForce",
                           .type = string{builtin::Constraint}});
    return uids;
}

AffineBodyRevoluteJointExternalBodyForce::AffineBodyRevoluteJointExternalBodyForce(const Json& config)
{
    m_config = config;
}

AffineBodyRevoluteJointExternalBodyForce::~AffineBodyRevoluteJointExternalBodyForce() = default;

void AffineBodyRevoluteJointExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                                         Float torque_v)
{
    vector<Float> torques(sc.edges().size(), torque_v);
    apply_to(sc, span{torques});
}

void AffineBodyRevoluteJointExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                                         span<Float> torques)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyRevoluteJointExternalBodyForce can only be applied to 1D simplicial complex (linemesh), "
                "but got {}D",
                sc.dim());

    auto size = sc.edges().size();
    UIPC_ASSERT(torques.size() == size,
                "Torques size mismatch: expected {}, got {}",
                size,
                torques.size());

    Base::apply_to(sc);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(uid && uid->view()[0] == 18,  // UID of AffineBodyRevoluteJoint
                "Simplicial complex does not have constitution uid 18. "
                "Please apply an AffineBodyRevoluteJoint before applying AffineBodyRevoluteJointExternalBodyForce");

    auto is_constrained = sc.edges().find<IndexT>(ExtForceIsConstrainedName);
    if(!is_constrained)
    {
        is_constrained = sc.edges().create<IndexT>(ExtForceIsConstrainedName, 0);
    }
    auto is_constrained_view = view(*is_constrained);
    std::ranges::fill(is_constrained_view, 0);

    auto external_torque = sc.edges().find<Float>("external_torque");
    if(!external_torque)
    {
        external_torque = sc.edges().create<Float>("external_torque", 0.0);
    }
    auto external_torque_view = view(*external_torque);
    std::ranges::copy(torques, external_torque_view.begin());
}

Json AffineBodyRevoluteJointExternalBodyForce::default_config()
{
    return Json::object();
}

U64 AffineBodyRevoluteJointExternalBodyForce::get_uid() const noexcept
{
    return ConstitutionUID;
}

}  // namespace uipc::constitution
