#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 18;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyRevoluteJoint",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

Json AffineBodyRevoluteJoint::default_config()
{
    return Json::object();
}

AffineBodyRevoluteJoint::AffineBodyRevoluteJoint(const Json& config)
{
    m_config = config;
}

AffineBodyRevoluteJoint::~AffineBodyRevoluteJoint() = default;

void AffineBodyRevoluteJoint::apply_to(geometry::SimplicialComplex& sc,
                                       span<SlotTuple>              geo_slots,
                                       Float strength_ratio_v)
{
    auto size = sc.edges().size();
    UIPC_ASSERT(size == geo_slots.size(),
                "The number of edges ({}) does not match the number of link pairs ({})",
                size,
                geo_slots.size());

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = sc.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto links = sc.edges().find<Vector2i>("links");
    if(!links)
    {
        links = sc.edges().create<Vector2i>("links", Vector2i{-1, -1});
    }
    auto links_view = view(*links);

    auto strength_ratio = sc.edges().find<Float>("strength_ratio");
    if(!strength_ratio)
    {
        strength_ratio = sc.edges().create<Float>("strength_ratio", 0.0);
    }
    std::ranges::fill(view(*strength_ratio), strength_ratio_v);

    std::ranges::transform(geo_slots,
                           links_view.begin(),
                           [](const SlotTuple& slot_tuple)
                           {
                               auto&& [l, r] = slot_tuple;

                               UIPC_ASSERT(l->geometry().instances().size() == 1,
                                           "Link must have exactly one instance, found {} instances",
                                           l->geometry().instances().size());
                               UIPC_ASSERT(r->geometry().instances().size() == 1,
                                           "Link must have exactly one instance, found {} instances",
                                           r->geometry().instances().size());

                               return Vector2i{l->id(), r->id()};
                           });
}

U64 AffineBodyRevoluteJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution