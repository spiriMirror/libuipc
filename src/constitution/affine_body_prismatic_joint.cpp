#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <uipc/common/enumerate.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 20;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyPrismaticJoint",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

Json AffineBodyPrismaticJoint::default_config()
{
    return Json::object();
}

AffineBodyPrismaticJoint::AffineBodyPrismaticJoint(const Json& config)
{
    m_config = config;
}

AffineBodyPrismaticJoint::~AffineBodyPrismaticJoint() = default;

void AffineBodyPrismaticJoint::apply_to(geometry::SimplicialComplex& sc,
                                        span<SlotTuple>              geo_slots,
                                        Float strength_ratio_v)
{
    UIPC_WARN_WITH_LOCATION(R"(AffineBodyPrismaticJoint::apply_to(SimplicialComplex&, span<SlotTuple>, Float) is deprecated. 
Ref: https://github.com/spiriMirror/libuipc/issues/260
)");

    auto size = sc.edges().size();
    UIPC_ASSERT(size == geo_slots.size(),
                "The number of edges ({}) does not match the number of link pairs ({})",
                size,
                geo_slots.size());

    // Convert old API to new API format
    vector<S<geometry::SimplicialComplexSlot>> l_geo_slots;
    vector<IndexT>                              l_instance_id;
    vector<S<geometry::SimplicialComplexSlot>> r_geo_slots;
    vector<IndexT>                              r_instance_id;

    l_geo_slots.reserve(size);
    l_instance_id.reserve(size);
    r_geo_slots.reserve(size);
    r_instance_id.reserve(size);

    for(const auto& slot_tuple : geo_slots)
    {
        auto&& [l, r] = slot_tuple;

        UIPC_ASSERT(l->geometry().instances().size() == 1,
                    "Link must have exactly one instance, found {} instances",
                    l->geometry().instances().size());
        UIPC_ASSERT(r->geometry().instances().size() == 1,
                    "Link must have exactly one instance, found {} instances",
                    r->geometry().instances().size());

        l_geo_slots.push_back(l);
        l_instance_id.push_back(0);
        r_geo_slots.push_back(r);
        r_instance_id.push_back(0);
    }

    vector<Float> strength_ratios(size, strength_ratio_v);
    apply_to(sc,
             span{l_geo_slots},
             span{l_instance_id},
             span{r_geo_slots},
             span{r_instance_id},
             span{strength_ratios});
}

void AffineBodyPrismaticJoint::apply_to(geometry::SimplicialComplex& edges,
                                       span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                       span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                       Float                                  strength_ratio)
{
    auto size = edges.edges().size();
    UIPC_ASSERT(size == l_geo_slots.size(),
                "The number of edges ({}) does not match the number of left geo slots ({})",
                size,
                l_geo_slots.size());
    UIPC_ASSERT(size == r_geo_slots.size(),
                "The number of edges ({}) does not match the number of right geo slots ({})",
                size,
                r_geo_slots.size());

    // Convert to instance-based API with instance 0 for all
    vector<IndexT> l_instance_id(size, 0);
    vector<IndexT> r_instance_id(size, 0);
    vector<Float>  strength_ratios(size, strength_ratio);

    apply_to(edges,
             span{l_geo_slots},
             span{l_instance_id},
             span{r_geo_slots},
             span{r_instance_id},
             span{strength_ratios});
}

void AffineBodyPrismaticJoint::apply_to(geometry::SimplicialComplex& edges,
                                       span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                       span<IndexT>                           l_instance_id,
                                       span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                       span<IndexT>                           r_instance_id,
                                       span<Float>                            strength_ratio)
{
    auto size = edges.edges().size();
    UIPC_ASSERT(size == l_geo_slots.size(),
                "The number of edges ({}) does not match the number of left geo slots ({})",
                size,
                l_geo_slots.size());
    UIPC_ASSERT(size == l_instance_id.size(),
                "The number of edges ({}) does not match the number of left instance IDs ({})",
                size,
                l_instance_id.size());
    UIPC_ASSERT(size == r_geo_slots.size(),
                "The number of edges ({}) does not match the number of right geo slots ({})",
                size,
                r_geo_slots.size());
    UIPC_ASSERT(size == r_instance_id.size(),
                "The number of edges ({}) does not match the number of right instance IDs ({})",
                size,
                r_instance_id.size());
    UIPC_ASSERT(size == strength_ratio.size(),
                "The number of edges ({}) does not match the number of strength ratios ({})",
                size,
                strength_ratio.size());

    auto uid = edges.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = edges.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto geo_ids = edges.edges().find<Vector2i>("geo_ids");
    if(!geo_ids)
    {
        geo_ids = edges.edges().create<Vector2i>("geo_ids", Vector2i{-1, -1});
    }
    auto geo_ids_view = view(*geo_ids);

    auto inst_ids = edges.edges().find<Vector2i>("inst_ids");
    if(!inst_ids)
    {
        inst_ids = edges.edges().create<Vector2i>("inst_ids", Vector2i{-1, -1});
    }
    auto inst_ids_view = view(*inst_ids);

    auto strength_ratio_attr = edges.edges().find<Float>("strength_ratio");
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = edges.edges().create<Float>("strength_ratio", 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio_attr);
    std::ranges::copy(strength_ratio, strength_ratio_view.begin());

    for(auto&& [i, l_slot] : enumerate(l_geo_slots))
    {
        auto r_slot = r_geo_slots[i];
        auto l_inst = l_instance_id[i];
        auto r_inst = r_instance_id[i];

        UIPC_ASSERT(l_inst >= 0 && l_inst < static_cast<IndexT>(l_slot->geometry().instances().size()),
                    "Left instance ID {} is out of range [0, {})",
                    l_inst,
                    l_slot->geometry().instances().size());
        UIPC_ASSERT(r_inst >= 0 && r_inst < static_cast<IndexT>(r_slot->geometry().instances().size()),
                    "Right instance ID {} is out of range [0, {})",
                    r_inst,
                    r_slot->geometry().instances().size());

        geo_ids_view[i]  = Vector2i{l_slot->id(), r_slot->id()};
        inst_ids_view[i] = Vector2i{l_inst, r_inst};
    }
}

U64 AffineBodyPrismaticJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution