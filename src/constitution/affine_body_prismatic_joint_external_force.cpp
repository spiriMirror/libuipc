#include <uipc/constitution/affine_body_prismatic_joint_external_force.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 667;

static constexpr std::string_view ExtForceIsConstrainedName = "external_force/is_constrained";

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyPrismaticJointExternalForce",
                           .type = string{builtin::Constraint}});
    return uids;
}

AffineBodyPrismaticJointExternalBodyForce::AffineBodyPrismaticJointExternalBodyForce(const Json& config)
{
    m_config = config;
}

AffineBodyPrismaticJointExternalBodyForce::~AffineBodyPrismaticJointExternalBodyForce() = default;

void AffineBodyPrismaticJointExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                                          Float force_v)
{
    vector<Float> forces(sc.edges().size(), force_v);
    apply_to(sc, span{forces});
}

void AffineBodyPrismaticJointExternalBodyForce::apply_to(geometry::SimplicialComplex& sc,
                                                          span<Float> forces)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyPrismaticJointExternalBodyForce can only be applied to 1D simplicial complex (linemesh), "
                "but got {}D",
                sc.dim());

    auto size = sc.edges().size();
    UIPC_ASSERT(forces.size() == size,
                "Forces size mismatch: expected {}, got {}",
                size,
                forces.size());

    Base::apply_to(sc);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(uid && uid->view()[0] == 20,  // UID of AffineBodyPrismaticJoint
                "Simplicial complex does not have constitution uid 20. "
                "Please apply an AffineBodyPrismaticJoint before applying AffineBodyPrismaticJointExternalBodyForce");

    auto is_constrained = sc.edges().find<IndexT>(ExtForceIsConstrainedName);
    if(!is_constrained)
    {
        is_constrained = sc.edges().create<IndexT>(ExtForceIsConstrainedName, 0);
    }
    auto is_constrained_view = view(*is_constrained);
    std::ranges::fill(is_constrained_view, 0);

    auto external_force = sc.edges().find<Float>("external_force");
    if(!external_force)
    {
        external_force = sc.edges().create<Float>("external_force", 0.0);
    }
    auto external_force_view = view(*external_force);
    std::ranges::copy(forces, external_force_view.begin());
}

Json AffineBodyPrismaticJointExternalBodyForce::default_config()
{
    return Json::object();
}

U64 AffineBodyPrismaticJointExternalBodyForce::get_uid() const noexcept
{
    return ConstitutionUID;
}

}  // namespace uipc::constitution
