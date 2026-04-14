#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <uipc/common/enumerate.h>

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

geometry::SimplicialComplex AffineBodyRevoluteJoint::create_geometry(
    span<const Vector3>                      position0s,
    span<const Vector3>                      position1s,
    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
    span<IndexT>                             l_instance_ids,
    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
    span<IndexT>                             r_instance_ids,
    span<Float>                              strength_ratios)
{
    auto N = position0s.size();
    UIPC_ASSERT_THROW(N == position1s.size(),
                "position0s({}) vs position1s({}) size mismatch",
                N,
                position1s.size());

    geometry::SimplicialComplex sc;
    sc.vertices().resize(2 * N);
    auto pos_view = view(sc.positions());

    sc.edges().resize(N);
    auto topo = sc.edges().create<Vector2i>(builtin::topo, Vector2i::Zero(), false);
    auto topo_view = view(*topo);

    for(SizeT i = 0; i < N; ++i)
    {
        pos_view[2 * i + 0] = position0s[i];
        pos_view[2 * i + 1] = position1s[i];
        topo_view[i] = Vector2i{static_cast<int>(2 * i), static_cast<int>(2 * i + 1)};
    }

    apply_to(sc, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);
    return sc;
}

geometry::SimplicialComplex AffineBodyRevoluteJoint::create_geometry(
    span<const Vector3>                      l_position0,
    span<const Vector3>                      l_position1,
    span<const Vector3>                      r_position0,
    span<const Vector3>                      r_position1,
    span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
    span<IndexT>                             l_instance_ids,
    span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
    span<IndexT>                             r_instance_ids,
    span<Float>                              strength_ratios)
{
    auto N = l_position0.size();
    UIPC_ASSERT_THROW(N == l_position1.size(),
                "l_position0({}) vs l_position1({}) size mismatch",
                N,
                l_position1.size());
    UIPC_ASSERT_THROW(N == r_position0.size(),
                "l_position0({}) vs r_position0({}) size mismatch",
                N,
                r_position0.size());
    UIPC_ASSERT_THROW(N == r_position1.size(),
                "l_position0({}) vs r_position1({}) size mismatch",
                N,
                r_position1.size());

    geometry::SimplicialComplex sc;
    sc.vertices().resize(2 * N);

    sc.edges().resize(N);
    auto topo = sc.edges().create<Vector2i>(builtin::topo, Vector2i::Zero(), false);
    auto topo_view = view(*topo);

    auto l_pos0_attr = sc.edges().create<Vector3>("l_position0", Vector3::Zero());
    auto l_pos1_attr = sc.edges().create<Vector3>("l_position1", Vector3::Zero());
    auto r_pos0_attr = sc.edges().create<Vector3>("r_position0", Vector3::Zero());
    auto r_pos1_attr = sc.edges().create<Vector3>("r_position1", Vector3::Zero());
    auto l_pos0_view = view(*l_pos0_attr);
    auto l_pos1_view = view(*l_pos1_attr);
    auto r_pos0_view = view(*r_pos0_attr);
    auto r_pos1_view = view(*r_pos1_attr);

    for(SizeT i = 0; i < N; ++i)
    {
        topo_view[i]    = Vector2i{static_cast<int>(2 * i), static_cast<int>(2 * i + 1)};
        l_pos0_view[i]  = l_position0[i];
        l_pos1_view[i]  = l_position1[i];
        r_pos0_view[i]  = r_position0[i];
        r_pos1_view[i]  = r_position1[i];
    }

    apply_to(sc, l_geo_slots, l_instance_ids, r_geo_slots, r_instance_ids, strength_ratios);
    return sc;
}

void AffineBodyRevoluteJoint::apply_to(geometry::SimplicialComplex& edges,
                                       span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                       span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                       Float strength_ratio)
{
    auto size = edges.edges().size();
    UIPC_ASSERT_THROW(size == l_geo_slots.size(),
                "The number of edges ({}) does not match the number of left geo slots ({})",
                size,
                l_geo_slots.size());
    UIPC_ASSERT_THROW(size == r_geo_slots.size(),
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

void AffineBodyRevoluteJoint::apply_to(geometry::SimplicialComplex& edges,
                                       span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                       span<IndexT> l_instance_ids,
                                       span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                       span<IndexT> r_instance_ids,
                                       span<Float>  strength_ratios)
{
    auto size = edges.edges().size();
    UIPC_ASSERT_THROW(size == l_geo_slots.size(),
                "The number of edges ({}) does not match the number of left geo slots ({})",
                size,
                l_geo_slots.size());
    UIPC_ASSERT_THROW(size == l_instance_ids.size(),
                "The number of edges ({}) does not match the number of left instance IDs ({})",
                size,
                l_instance_ids.size());
    UIPC_ASSERT_THROW(size == r_geo_slots.size(),
                "The number of edges ({}) does not match the number of right geo slots ({})",
                size,
                r_geo_slots.size());
    UIPC_ASSERT_THROW(size == r_instance_ids.size(),
                "The number of edges ({}) does not match the number of right instance IDs ({})",
                size,
                r_instance_ids.size());
    UIPC_ASSERT_THROW(size == strength_ratios.size(),
                "The number of edges ({}) does not match the number of strength ratios ({})",
                size,
                strength_ratios.size());

    auto uid = edges.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = edges.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto l_geo_id_attr = edges.edges().find<IndexT>("l_geo_id");
    if(!l_geo_id_attr)
        l_geo_id_attr = edges.edges().create<IndexT>("l_geo_id", IndexT{-1});
    auto l_geo_id_view = view(*l_geo_id_attr);

    auto r_geo_id_attr = edges.edges().find<IndexT>("r_geo_id");
    if(!r_geo_id_attr)
        r_geo_id_attr = edges.edges().create<IndexT>("r_geo_id", IndexT{-1});
    auto r_geo_id_view = view(*r_geo_id_attr);

    auto l_inst_id_attr = edges.edges().find<IndexT>("l_inst_id");
    if(!l_inst_id_attr)
        l_inst_id_attr = edges.edges().create<IndexT>("l_inst_id", IndexT{-1});
    auto l_inst_id_view = view(*l_inst_id_attr);

    auto r_inst_id_attr = edges.edges().find<IndexT>("r_inst_id");
    if(!r_inst_id_attr)
        r_inst_id_attr = edges.edges().create<IndexT>("r_inst_id", IndexT{-1});
    auto r_inst_id_view = view(*r_inst_id_attr);

    auto strength_ratio_attr = edges.edges().find<Float>("strength_ratio");
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = edges.edges().create<Float>("strength_ratio", 0.0);
    }
    auto strength_ratio_view = view(*strength_ratio_attr);
    std::ranges::copy(strength_ratios, strength_ratio_view.begin());

    auto angle = edges.edges().find<Float>("angle");
    if(!angle)
    {
        angle = edges.edges().create<Float>("angle", 0.0);
    }
    auto angle_view = view(*angle);
    std::ranges::fill(angle_view, 0.0);

    auto init_angle = edges.edges().find<Float>("init_angle");
    if(!init_angle)
    {
        init_angle = edges.edges().create<Float>("init_angle", 0.0);
    }
    auto init_angle_view = view(*init_angle);
    std::ranges::fill(init_angle_view, 0.0);

    for(auto&& [i, l_slot] : enumerate(l_geo_slots))
    {
        auto r_slot = r_geo_slots[i];
        auto l_inst = l_instance_ids[i];
        auto r_inst = r_instance_ids[i];

        UIPC_ASSERT_THROW(l_inst >= 0
                        && l_inst < static_cast<IndexT>(
                               l_slot->geometry().instances().size()),
                    "Left instance ID {} is out of range [0, {})",
                    l_inst,
                    l_slot->geometry().instances().size());
        UIPC_ASSERT_THROW(r_inst >= 0
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

U64 AffineBodyRevoluteJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution