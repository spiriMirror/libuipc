#include <uipc/constitution/affine_body_free_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>
#include <Eigen/Geometry>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 33;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyFreeJoint",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}

Json AffineBodyFreeJoint::default_config()
{
    return Json::object();
}

AffineBodyFreeJoint::AffineBodyFreeJoint(const Json& config)
{
    m_config = config;
}

AffineBodyFreeJoint::~AffineBodyFreeJoint() = default;

geometry::SimplicialComplex AffineBodyFreeJoint::create_geometry(
    span<S<geometry::SimplicialComplexSlot>> geo_slots, span<IndexT> instance_ids)
{
    auto N = geo_slots.size();
    UIPC_ASSERT(N == instance_ids.size(),
                "geo_slots({}) vs instance_ids({}) size mismatch",
                N,
                instance_ids.size());

    geometry::SimplicialComplex sc;
    sc.vertices().resize(6 * N);
    auto pos_view = view(sc.positions());

    for(SizeT i = 0; i < N; ++i)
    {
        auto l_sc = geo_slots[i]->geometry().as<geometry::SimplicialComplex>();
        UIPC_ASSERT(l_sc, "geo_slots[{}] does not hold a SimplicialComplex", i);
        UIPC_ASSERT(instance_ids[i] >= 0
                        && instance_ids[i]
                               < static_cast<IndexT>(l_sc->transforms().view().size()),
                    "Instance ID {} out of range [0, {}) in create_geometry",
                    instance_ids[i],
                    l_sc->transforms().view().size());

        Transform T{l_sc->transforms().view()[instance_ids[i]]};
        Vector3   center = T.translation();

        for(int dof = 0; dof < 6; ++dof)
        {
            pos_view[6 * i + dof] = center;
        }
    }

    apply_to(sc, geo_slots, instance_ids);
    return sc;
}

void AffineBodyFreeJoint::apply_to(geometry::SimplicialComplex& sc,
                                   span<S<geometry::SimplicialComplexSlot>> geo_slots,
                                   span<IndexT> instance_ids)
{
    auto N = geo_slots.size();
    UIPC_ASSERT(N == instance_ids.size(),
                "geo_slots({}) vs instance_ids({}) size mismatch",
                N,
                instance_ids.size());
    UIPC_ASSERT(sc.vertices().size() == 6 * N,
                "Vertex count ({}) must equal 6 * N ({})",
                sc.vertices().size(),
                6 * N);

    auto uid = sc.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
    {
        uid = sc.meta().create<U64>(builtin::constitution_uid, 0);
    }
    view(*uid)[0] = this->uid();

    auto l_geo_id_attr = sc.vertices().find<IndexT>("l_geo_id");
    if(!l_geo_id_attr)
    {
        l_geo_id_attr = sc.vertices().create<IndexT>("l_geo_id", IndexT{-1});
    }
    auto l_geo_id_view = view(*l_geo_id_attr);

    auto l_inst_id_attr = sc.vertices().find<IndexT>("l_inst_id");
    if(!l_inst_id_attr)
    {
        l_inst_id_attr = sc.vertices().create<IndexT>("l_inst_id", IndexT{-1});
    }
    auto l_inst_id_view = view(*l_inst_id_attr);

    auto dof_type_attr = sc.vertices().find<IndexT>("dof_type");
    if(!dof_type_attr)
    {
        dof_type_attr = sc.vertices().create<IndexT>("dof_type", IndexT{-1});
    }
    auto dof_type_view = view(*dof_type_attr);

    for(SizeT i = 0; i < N; ++i)
    {
        auto slot    = geo_slots[i];
        auto inst_id = instance_ids[i];

        UIPC_ASSERT(inst_id >= 0
                        && inst_id < static_cast<IndexT>(
                               slot->geometry().instances().size()),
                    "Instance ID {} is out of range [0, {})",
                    inst_id,
                    slot->geometry().instances().size());

        for(int dof = 0; dof < 6; ++dof)
        {
            SizeT vi           = 6 * i + dof;
            l_geo_id_view[vi]  = slot->id();
            l_inst_id_view[vi] = inst_id;
            dof_type_view[vi]  = dof;
        }
    }
}

U64 AffineBodyFreeJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
