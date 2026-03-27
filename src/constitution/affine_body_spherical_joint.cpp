#include <uipc/constitution/affine_body_spherical_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 26;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodySphericalJoint",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

Json AffineBodySphericalJoint::default_config()
{
    return Json::object();
}

AffineBodySphericalJoint::AffineBodySphericalJoint(const Json& config)
{
    m_config = config;
}

AffineBodySphericalJoint::~AffineBodySphericalJoint() = default;

void AffineBodySphericalJoint::apply_to(geometry::SimplicialComplex& sc,
                                        span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                        span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                        span<Vector3> r_local_pos,
                                        Float         strength_ratio)
{
    auto size = l_geo_slots.size();
    UIPC_ASSERT(size == r_geo_slots.size(),
                "The number of left geo slots ({}) does not match the number of right geo slots ({})",
                l_geo_slots.size(),
                r_geo_slots.size());

    vector<IndexT> l_instance_ids(size, 0);
    vector<IndexT> r_instance_ids(size, 0);
    vector<Float>  strength_ratios(size, strength_ratio);

    apply_to(sc,
             span{l_geo_slots},
             span{l_instance_ids},
             span{r_geo_slots},
             span{r_instance_ids},
             span{r_local_pos},
             span{strength_ratios});
}

void AffineBodySphericalJoint::apply_to(geometry::SimplicialComplex& sc,
                                        span<S<geometry::SimplicialComplexSlot>> l_geo_slots,
                                        span<IndexT> l_instance_ids,
                                        span<S<geometry::SimplicialComplexSlot>> r_geo_slots,
                                        span<IndexT>  r_instance_ids,
                                        span<Vector3> r_local_pos,
                                        span<Float>   strength_ratios)
{
    auto size = l_geo_slots.size();
    UIPC_ASSERT(size == r_geo_slots.size(),
                "Size mismatch: l_geo_slots ({}) vs r_geo_slots ({})",
                size,
                r_geo_slots.size());
    UIPC_ASSERT(size == l_instance_ids.size(),
                "Size mismatch: l_geo_slots ({}) vs l_instance_ids ({})",
                size,
                l_instance_ids.size());
    UIPC_ASSERT(size == r_instance_ids.size(),
                "Size mismatch: l_geo_slots ({}) vs r_instance_ids ({})",
                size,
                r_instance_ids.size());
    UIPC_ASSERT(size == r_local_pos.size(),
                "Size mismatch: l_geo_slots ({}) vs r_local_pos ({})",
                size,
                r_local_pos.size());
    UIPC_ASSERT(size == strength_ratios.size(),
                "Size mismatch: l_geo_slots ({}) vs strength_ratios ({})",
                size,
                strength_ratios.size());

    // Build vertices: 2 per joint
    //   vertex 0: body0's world-space center (for edge visualization)
    //   vertex 1: anchor point in world space = RT * r_local_pos (body1's attachment)
    sc.vertices().resize(2 * size);
    auto pos_view = view(sc.positions());

    // Build edges: 1 per joint
    sc.edges().resize(size);
    auto topo = sc.edges().create<Vector2i>(builtin::topo, Vector2i::Zero(), false);
    auto topo_view = view(*topo);

    for(SizeT i = 0; i < size; ++i)
    {
        auto l_sc = l_geo_slots[i]->geometry().as<geometry::SimplicialComplex>();
        auto r_sc = r_geo_slots[i]->geometry().as<geometry::SimplicialComplex>();

        Transform LT{l_sc->transforms().view()[l_instance_ids[i]]};
        Transform RT{r_sc->transforms().view()[r_instance_ids[i]]};

        // vertex 0: body0's center (visualization only)
        pos_view[2 * i + 0] = LT.translation();
        // vertex 1: anchor = body1's attachment point in world space
        pos_view[2 * i + 1] = RT * r_local_pos[i];
        topo_view[i] = Vector2i{static_cast<int>(2 * i), static_cast<int>(2 * i + 1)};
    }

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = sc.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto geo_ids = sc.edges().find<Vector2i>("geo_ids");
    if(!geo_ids)
    {
        geo_ids = sc.edges().create<Vector2i>("geo_ids", Vector2i{-1, -1});
    }
    auto geo_ids_view = view(*geo_ids);

    auto inst_ids = sc.edges().find<Vector2i>("inst_ids");
    if(!inst_ids)
    {
        inst_ids = sc.edges().create<Vector2i>("inst_ids", Vector2i{-1, -1});
    }
    auto inst_ids_view = view(*inst_ids);

    auto strength_ratio_attr = sc.edges().find<Float>("strength_ratio");
    if(!strength_ratio_attr)
    {
        strength_ratio_attr = sc.edges().create<Float>("strength_ratio", 0.0);
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

        geo_ids_view[i]  = Vector2i{l_slot->id(), r_slot->id()};
        inst_ids_view[i] = Vector2i{l_inst, r_inst};
    }
}

U64 AffineBodySphericalJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
