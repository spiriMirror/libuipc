#include <uipc/constitution/affine_body_prismatic_joint_limit.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID            = 669;
static constexpr U64 PrismaticJointUID          = 20;
static constexpr char LimitLowerName[]          = "limit/lower";
static constexpr char LimitUpperName[]          = "limit/upper";
static constexpr char LimitStrengthName[]       = "limit/strength";

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyPrismaticJointLimit",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

AffineBodyPrismaticJointLimit::AffineBodyPrismaticJointLimit(const Json& config)
    : m_config(config)
{
}

AffineBodyPrismaticJointLimit::~AffineBodyPrismaticJointLimit() = default;

Json AffineBodyPrismaticJointLimit::default_config()
{
    return Json::object();
}

void AffineBodyPrismaticJointLimit::apply_to(geometry::SimplicialComplex& sc,
                                             Float                        lower,
                                             Float                        upper,
                                             Float                        strength)
{
    auto edge_count = sc.edges().size();
    vector<Float> lowers(edge_count, lower);
    vector<Float> uppers(edge_count, upper);
    vector<Float> strengths(edge_count, strength);
    apply_to(sc, span{lowers}, span{uppers}, span{strengths});
}

void AffineBodyPrismaticJointLimit::apply_to(geometry::SimplicialComplex& sc,
                                             span<Float>                  lowers,
                                             span<Float>                  uppers,
                                             span<Float>                  strengths)
{
    UIPC_ASSERT(sc.dim() == 1,
                "AffineBodyPrismaticJointLimit can only be applied to 1D simplicial complex (linemesh), but got {}D",
                sc.dim());

    auto base_uid = sc.meta().find<U64>(builtin::constitution_uid);
    UIPC_ASSERT(base_uid, "AffineBodyPrismaticJointLimit requires `meta.constitution_uid` on joint mesh");
    UIPC_ASSERT(base_uid->view()[0] == PrismaticJointUID,
                "AffineBodyPrismaticJointLimit must be applied on a prismatic joint mesh with constitution UID={}, but got {}",
                PrismaticJointUID,
                base_uid->view()[0]);

    auto edge_count = sc.edges().size();
    UIPC_ASSERT(lowers.size() == edge_count,
                "Lower limit size mismatch: expected {}, got {}",
                edge_count,
                lowers.size());
    UIPC_ASSERT(uppers.size() == edge_count,
                "Upper limit size mismatch: expected {}, got {}",
                edge_count,
                uppers.size());
    UIPC_ASSERT(strengths.size() == edge_count,
                "Strength size mismatch: expected {}, got {}",
                edge_count,
                strengths.size());

    for(auto&& [i, l] : enumerate(lowers))
    {
        UIPC_ASSERT(l <= uppers[i],
                    "Invalid limit range at edge {}: lower ({}) must be <= upper ({})",
                    i,
                    l,
                    uppers[i]);
    }

    Base::apply_to(sc);

    auto lower_attr = sc.edges().find<Float>(LimitLowerName);
    if(!lower_attr)
        lower_attr = sc.edges().create<Float>(LimitLowerName, 0.0f);

    auto upper_attr = sc.edges().find<Float>(LimitUpperName);
    if(!upper_attr)
        upper_attr = sc.edges().create<Float>(LimitUpperName, 0.0f);

    auto strength_attr = sc.edges().find<Float>(LimitStrengthName);
    if(!strength_attr)
        strength_attr = sc.edges().create<Float>(LimitStrengthName, 1.0f);

    auto lower_view    = view(*lower_attr);
    auto upper_view    = view(*upper_attr);
    auto strength_view = view(*strength_attr);

    std::ranges::copy(lowers, lower_view.begin());
    std::ranges::copy(uppers, upper_view.begin());
    std::ranges::copy(strengths, strength_view.begin());
}

U64 AffineBodyPrismaticJointLimit::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
