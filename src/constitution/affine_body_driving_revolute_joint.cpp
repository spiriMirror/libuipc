#include <uipc/constitution/affine_body_driving_revolute_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 19;
static constexpr std::string_view DrivingStrengthName = "driving/strength_ratio";
static constexpr std::string_view DrivingIsConstrainedName = "driving/is_constrained";
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
    vector<Float> strength_ratios(sc.edges().size(), strength_ratio_v);
    apply_to(sc, span{strength_ratios});
}

void AffineBodyDrivingRevoluteJoint::apply_to(geometry::SimplicialComplex& sc,
                                              span<Float> strength_ratio)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyDrivingRevoluteJoint can only be applied to 1D simplicial complex (linemesh), "
                "but got {}D",
                sc.dim());

    auto size = sc.edges().size();
    UIPC_ASSERT(strength_ratio.size() == size,
                "Strength ratio size mismatch: expected {}, got {}",
                size,
                strength_ratio.size());
    Base::apply_to(sc);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(uid && uid->view()[0] == 18,  // UID of AffineBodyRevoluteJoint
                "Simplicial complex does not have constitution uid. "
                "Please apply an AffineBodyRevoluteJoint before applying AffineBodyDrivingRevoluteJoint");

    auto is_constrained = sc.edges().find<IndexT>(DrivingIsConstrainedName);
    if(!is_constrained)
    {
        is_constrained = sc.edges().create<IndexT>(DrivingIsConstrainedName, 0);
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

    auto strength_ratio_attr = sc.edges().find<Float>(DrivingStrengthName);
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = sc.edges().create<Float>(DrivingStrengthName, 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio_attr);
    std::ranges::copy(strength_ratio, strength_ratio_view.begin());

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