#include <uipc/constitution/affine_body_fixed_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 25;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyFixedJoint",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

Json AffineBodyFixedJoint::default_config()
{
    return Json::object();
}

AffineBodyFixedJoint::AffineBodyFixedJoint(const Json& config)
{
    m_config = config;
}

AffineBodyFixedJoint::~AffineBodyFixedJoint() = default;

geometry::SimplicialComplex AffineBodyFixedJoint::create_geometry(
    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
    span<IndexT>                             l_instance_ids,
    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
    span<IndexT>                             r_instance_ids,
    span<Float>                              strength_ratios)
{
    auto N = l_geo_slots.size();
    UIPC_ASSERT(N == r_geo_slots.size(),
                "l_geo_slots({}) vs r_geo_slots({}) size mismatch",
                N,
                r_geo_slots.size());
    UIPC_ASSERT(N == l_instance_ids.size(),
                "l_geo_slots({}) vs l_instance_ids({}) size mismatch",
                N,
                l_instance_ids.size());

    geometry::SimplicialComplex sc;
    sc.vertices().resize(N);
    auto pos_view = view(sc.positions());

    for(SizeT i = 0; i < N; ++i)
    {
        auto l_sc = l_geo_slots[i]->geometry().as<geometry::SimplicialComplex>();
        auto r_sc = r_geo_slots[i]->geometry().as<geometry::SimplicialComplex>();

        Transform LT{l_sc->transforms().view()[l_instance_ids[i]]};
        Transform RT{r_sc->transforms().view()[r_instance_ids[i]]};

        pos_view[i] = (LT.translation() + RT.translation()) * 0.5;
    }

    apply_to(sc, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);
    return sc;
}

geometry::SimplicialComplex AffineBodyFixedJoint::create_geometry(
    span<const Vector3>                      l_positions,
    span<const Vector3>                      r_positions,
    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
    span<IndexT>                             l_instance_ids,
    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
    span<IndexT>                             r_instance_ids,
    span<Float>                              strength_ratios)
{
    auto N = l_positions.size();
    UIPC_ASSERT(N == r_positions.size(),
                "l_positions({}) vs r_positions({}) size mismatch",
                N,
                r_positions.size());

    geometry::SimplicialComplex sc;
    sc.vertices().resize(N);

    auto pos0_attr = sc.vertices().create<Vector3>("l_position", Vector3::Zero());
    auto pos1_attr = sc.vertices().create<Vector3>("r_position", Vector3::Zero());
    auto pos0_view = view(*pos0_attr);
    auto pos1_view = view(*pos1_attr);

    for(SizeT i = 0; i < N; ++i)
    {
        pos0_view[i] = l_positions[i];
        pos1_view[i] = r_positions[i];
    }

    apply_to(sc, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);
    return sc;
}

void AffineBodyFixedJoint::apply_to(geometry::SimplicialComplex& sc,
                                    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                    Float strength_ratio)
{
    auto size = l_geo_slots.size();
    UIPC_ASSERT(size == r_geo_slots.size(),
                "The number of left geo slots ({}) does not match the number of right geo slots ({})",
                l_geo_slots.size(),
                r_geo_slots.size());

    vector<IndexT> l_instance_id(size, 0);
    vector<IndexT> r_instance_id(size, 0);
    vector<Float>  strength_ratios(size, strength_ratio);

    apply_to(sc,
             span{l_geo_slots},
             span{l_instance_id},
             span{r_geo_slots},
             span{r_instance_id},
             span{strength_ratios});
}

void AffineBodyFixedJoint::apply_to(geometry::SimplicialComplex& sc,
                                    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                    span<IndexT> l_instance_ids,
                                    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                    span<IndexT> r_instance_ids,
                                    span<Float>  strength_ratios)
{
    auto size = l_geo_slots.size();
    UIPC_ASSERT(size == r_geo_slots.size(),
                "Size mismatch: l_geo_slots({}) vs r_geo_slots({})",
                l_geo_slots.size(),
                r_geo_slots.size());
    UIPC_ASSERT(size == l_instance_ids.size(),
                "Size mismatch: l_geo_slots({}) vs l_instance_ids({})",
                l_geo_slots.size(),
                l_instance_ids.size());
    UIPC_ASSERT(size == r_instance_ids.size(),
                "Size mismatch: r_geo_slots({}) vs r_instance_ids({})",
                r_geo_slots.size(),
                r_instance_ids.size());
    UIPC_ASSERT(size == strength_ratios.size(),
                "Size mismatch: l_geo_slots({}) vs strength_ratios({})",
                l_geo_slots.size(),
                strength_ratios.size());

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = sc.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto l_geo_id_attr = sc.vertices().find<IndexT>("l_geo_id");
    if(!l_geo_id_attr)
        l_geo_id_attr = sc.vertices().create<IndexT>("l_geo_id", IndexT{-1});
    auto l_geo_id_view = view(*l_geo_id_attr);

    auto r_geo_id_attr = sc.vertices().find<IndexT>("r_geo_id");
    if(!r_geo_id_attr)
        r_geo_id_attr = sc.vertices().create<IndexT>("r_geo_id", IndexT{-1});
    auto r_geo_id_view = view(*r_geo_id_attr);

    auto l_inst_id_attr = sc.vertices().find<IndexT>("l_inst_id");
    if(!l_inst_id_attr)
        l_inst_id_attr = sc.vertices().create<IndexT>("l_inst_id", IndexT{-1});
    auto l_inst_id_view = view(*l_inst_id_attr);

    auto r_inst_id_attr = sc.vertices().find<IndexT>("r_inst_id");
    if(!r_inst_id_attr)
        r_inst_id_attr = sc.vertices().create<IndexT>("r_inst_id", IndexT{-1});
    auto r_inst_id_view = view(*r_inst_id_attr);

    auto strength_ratio_attr = sc.vertices().find<Float>("strength_ratio");
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = sc.vertices().create<Float>("strength_ratio", 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio_attr);
    std::ranges::copy(strength_ratios, strength_ratio_view.begin());

    for(auto&& [i, l_slot] : enumerate(l_geo_slots))
    {
        auto r_slot = r_geo_slots[i];
        auto l_inst = l_instance_ids[i];
        auto r_inst = r_instance_ids[i];

        UIPC_ASSERT(l_inst >= 0
                        && l_inst < static_cast<IndexT>(
                               l_slot->geometry().instances().size()),
                    "Left instance ID {} is out of range [0, {})",
                    l_inst,
                    l_slot->geometry().instances().size());
        UIPC_ASSERT(r_inst >= 0
                        && r_inst < static_cast<IndexT>(
                               r_slot->geometry().instances().size()),
                    "Right instance ID {} is out of range [0, {})",
                    r_inst,
                    r_slot->geometry().instances().size());

        l_geo_id_view[i]  = l_slot->id();
        r_geo_id_view[i]  = r_slot->id();
        l_inst_id_view[i] = l_inst;
        r_inst_id_view[i] = r_inst;
    }
}

U64 AffineBodyFixedJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
