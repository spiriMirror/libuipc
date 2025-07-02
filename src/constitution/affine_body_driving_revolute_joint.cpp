#include <uipc/constitution/affine_body_driving_revolute_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 19;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyDrivingRevoluteJoint",
                           .type = string{builtin::Constraint}});
    return uids;
}

AffineBodyDrivingRevoluteJoint::AffineBodyDrivingRevoluteJoint(const Json& config)
{
    m_config = config;
}

AffineBodyDrivingRevoluteJoint::~AffineBodyDrivingRevoluteJoint() = default;

void AffineBodyDrivingRevoluteJoint::apply_to(geometry::SimplicialComplex& sc, Float strength_ratio_v)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyDrivingRevoluteJoint can only be applied to 1D simplicial complex (linemesh), "
                "but got {}D",
                sc.dim());

    Base::apply_to(sc);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(uid && uid->view()[0] == 18,  // UID of AffineBodyRevoluteJoint
                "Simplicial complex does not have constitution uid. "
                "Please apply an AffineBodyRevoluteJoint before applying AffineBodyDrivingRevoluteJoint");


    auto aim_angle = sc.edges().find<Float>("aim_angle");
    if(!aim_angle)
    {
        aim_angle = sc.edges().create<Float>("aim_angle", 0.0);
    }
    auto aim_angle_view = view(*aim_angle);
    std::ranges::fill(aim_angle_view, 0.0);

    auto angle = sc.edges().find<Float>("angle");
    if(!angle)
    {
        angle = sc.edges().create<Float>("angle", 0.0);
    }
    auto angle_view = view(*angle);
    std::ranges::fill(angle_view, 0.0);

    auto init_angle = sc.edges().find<Float>("init_angle");
    if(!init_angle)
    {
        init_angle = sc.edges().create<Float>("init_angle");
    }
    auto init_angle_view = view(*init_angle);
    std::ranges::fill(init_angle_view, 0.0);

    auto is_constrained = sc.edges().find<IndexT>(builtin::is_constrained);
    if(!is_constrained)
    {
        is_constrained = sc.edges().create<IndexT>(builtin::is_constrained, 0);
    }
    auto is_constrained_view = view(*is_constrained);
    std::ranges::fill(is_constrained_view, 0);

    auto is_passive = sc.edges().find<IndexT>("is_passive");
    if(!is_passive)
    {
        is_passive = sc.edges().create<IndexT>("is_passive", 0);
    }
    auto is_passive_view = view(*is_passive);
    std::ranges::fill(is_passive_view, 0);

    auto strength_ratio = sc.edges().find<Float>("driving/strength_ratio");
    if(!strength_ratio)
    {
        strength_ratio = sc.edges().create<Float>("driving/strength_ratio", 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio);
    std::ranges::fill(strength_ratio_view, strength_ratio_v);
}

Json AffineBodyDrivingRevoluteJoint::default_config()
{
    return Json::object();
}

U64 AffineBodyDrivingRevoluteJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}

}  // namespace uipc::constitution