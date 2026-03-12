#include <uipc/constitution/affine_body_driving_prismatic_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 21;

static constexpr std::string_view DrivingStrengthName = "driving/strength_ratio";
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyDrivingPrismaticJoint",
                           .type = string{builtin::Constraint}});
    return uids;
}

AffineBodyDrivingPrismaticJoint::AffineBodyDrivingPrismaticJoint(const Json& config)
{
    m_config = config;
}

AffineBodyDrivingPrismaticJoint::~AffineBodyDrivingPrismaticJoint() = default;


void AffineBodyDrivingPrismaticJoint::apply_to(geometry::SimplicialComplex& sc, Float strength_ratio_v)
{
    vector<Float> strength_ratios(sc.edges().size(), strength_ratio_v);
    apply_to(sc, span{strength_ratios});
}


void AffineBodyDrivingPrismaticJoint::apply_to(geometry::SimplicialComplex& sc,
                                               span<Float> strength_ratio)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyDrivingPrismaticJoint can only be applied to 1D simplicial complex (linemesh), "
                "but got {}D",
                sc.dim());

    auto size = sc.edges().size();
    UIPC_ASSERT(strength_ratio.size() == size,
                "Strength ratio size mismatch: expected {}, got {}",
                size,
                strength_ratio.size());

    Base::apply_to(sc);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(uid && uid->view()[0] == 20,  // UID of AffineBodyPrismaticJoint
                "Simplicial complex does not have constitution uid. "
                "Please apply an AffineBodyPrismaticJoint before applying AffineBodyDrivingPrismaticJoint");

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

    auto strength_ratio_attr = sc.edges().find<Float>(DrivingStrengthName);
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = sc.edges().create<Float>(DrivingStrengthName, 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio_attr);
    std::ranges::copy(strength_ratio, strength_ratio_view.begin());

    auto distance = sc.edges().find<Float>("distance");
    if(!distance)
    {
        distance = sc.edges().create<Float>("distance", 0.0);
    }
    auto distance_view = view(*distance);
    std::ranges::fill(distance_view, 0.0);

    auto init_distances = sc.edges().find<Float>("init_distance");
    if(!init_distances)
    {
        init_distances = sc.edges().create<Float>("init_distance", 0);
    }
    auto init_distances_view = view(*init_distances);
    std::ranges::fill(init_distances_view, 0);

    auto aim_distances = sc.edges().find<Float>("aim_distance");
    if(!aim_distances)
    {
        aim_distances = sc.edges().create<Float>("aim_distance", 0);
    }
    auto aim_distances_view = view(*aim_distances);
    std::ranges::fill(aim_distances_view, 0);
}

Json AffineBodyDrivingPrismaticJoint::default_config()
{
    return Json::object();
}

U64 AffineBodyDrivingPrismaticJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}

}  // namespace uipc::constitution