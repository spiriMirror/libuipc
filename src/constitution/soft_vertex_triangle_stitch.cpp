#include <uipc/constitution/soft_vertex_triangle_stitch.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/attribute_friend.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 30;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    uipc::list<UIDInfo> uids;

    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "SoftVertexTriangleStitch",
                           .type = string{builtin::InterPrimitive}});

    return uids;
}

SoftVertexTriangleStitch::SoftVertexTriangleStitch(const Json& config)
{
    m_config = config;
}


static geometry::Geometry create_geometry_impl(const SoftVertexTriangleStitch::SlotTuple& aim_geo_slots,
                                               const SoftVertexTriangleStitch::SlotTuple& rest_geo_slots,
                                               span<const Vector2i> stitched_vert_tri_ids,
                                               Float mu,
                                               Float lambda,
                                               Float min_separate_distance);

static void validate_stitched_vert_tri_ids(const SoftVertexTriangleStitch::SlotTuple& aim_geo_slots,
                                           const SoftVertexTriangleStitch::SlotTuple& rest_geo_slots,
                                           span<const Vector2i> stitched_vert_tri_ids)
{
    auto&& [aim_v_slot, aim_tri_slot] = aim_geo_slots;
    auto&& [rest_v_slot, rest_tri_slot] = rest_geo_slots;

    UIPC_ASSERT(aim_v_slot, "SoftVertexTriangleStitch: first aim slot is null.");
    UIPC_ASSERT(aim_tri_slot, "SoftVertexTriangleStitch: second aim slot is null.");
    UIPC_ASSERT(rest_v_slot, "SoftVertexTriangleStitch: first rest slot is null.");
    UIPC_ASSERT(rest_tri_slot, "SoftVertexTriangleStitch: second rest slot is null.");

    UIPC_ASSERT(aim_v_slot->geometry().instances().size() == 1,
                "SoftVertexTriangleStitch expects first aim geometry to have exactly one instance, found {}.",
                aim_v_slot->geometry().instances().size());
    UIPC_ASSERT(aim_tri_slot->geometry().instances().size() == 1,
                "SoftVertexTriangleStitch expects second aim geometry to have exactly one instance, found {}.",
                aim_tri_slot->geometry().instances().size());
    UIPC_ASSERT(rest_v_slot->geometry().instances().size() == 1,
                "SoftVertexTriangleStitch expects first rest geometry to have exactly one instance, found {}.",
                rest_v_slot->geometry().instances().size());
    UIPC_ASSERT(rest_tri_slot->geometry().instances().size() == 1,
                "SoftVertexTriangleStitch expects second rest geometry to have exactly one instance, found {}.",
                rest_tri_slot->geometry().instances().size());

    const auto vert_count = aim_v_slot->geometry().vertices().size();
    const auto tri_count  = aim_tri_slot->geometry().triangles().size();

    for(auto&& [pair_idx, pair] : enumerate(stitched_vert_tri_ids))
    {
        auto v_id   = pair.x();
        auto tri_id = pair.y();

        UIPC_ASSERT(v_id >= 0 && v_id < vert_count,
                    "SoftVertexTriangleStitch pair[{}].x={} out of range [0, {}) for first geometry slot id {}.",
                    pair_idx,
                    v_id,
                    vert_count,
                    aim_v_slot->id());
        UIPC_ASSERT(tri_id >= 0 && tri_id < tri_count,
                    "SoftVertexTriangleStitch pair[{}].y={} out of range [0, {}) for second geometry slot id {}.",
                    pair_idx,
                    tri_id,
                    tri_count,
                    aim_tri_slot->id());
    }
}


geometry::Geometry SoftVertexTriangleStitch::create_geometry(const SlotTuple& aim_geo_slots,
                                                             const SlotTuple& rest_geo_slots,
                                                             span<const Vector2i> stitched_vert_tri_ids,
                                                             const ElasticModuli& moduli,
                                                             Float min_separate_distance) const
{
    return create_geometry_impl(aim_geo_slots,
                                rest_geo_slots,
                                stitched_vert_tri_ids,
                                moduli.mu(),
                                moduli.lambda(),
                                min_separate_distance);
}

geometry::Geometry SoftVertexTriangleStitch::create_geometry(const SlotTuple& aim_geo_slots,
                                                             const SlotTuple& rest_geo_slots,
                                                             const geometry::Geometry& pair_geometry,
                                                             const ElasticModuli& moduli,
                                                             Float min_separate_distance) const
{
    auto topo_slot = pair_geometry.instances().find<Vector2i>(builtin::topo);
    UIPC_ASSERT(topo_slot, "pair_geometry must have instances with topo (Vector2i) attribute");
    auto topo_view = topo_slot->view();
    return create_geometry_impl(
        aim_geo_slots, rest_geo_slots, topo_view, moduli.mu(), moduli.lambda(), min_separate_distance);
}


static geometry::Geometry create_geometry_impl(const SoftVertexTriangleStitch::SlotTuple& aim_geo_slots,
                                               const SoftVertexTriangleStitch::SlotTuple& rest_geo_slots,
                                               span<const Vector2i> stitched_vert_tri_ids,
                                               Float mu,
                                               Float lambda,
                                               Float min_separate_distance)
{
    validate_stitched_vert_tri_ids(aim_geo_slots, rest_geo_slots, stitched_vert_tri_ids);

    geometry::Geometry geo;

    auto uids      = geo.meta().create<U64>(builtin::constitution_uid);
    view(*uids)[0] = ConstitutionUID;

    auto geo_ids = geo.meta().create<Vector2i>("geo_ids");
    {
        auto&& [l, r] = aim_geo_slots;
        view(*geo_ids)[0] = Vector2i{l->id(), r->id()};
    }

    const auto& rest1    = std::get<1>(rest_geo_slots)->geometry();
    auto        tri_topo = rest1.triangles().topo().view();

    geo.instances().resize(stitched_vert_tri_ids.size());

    auto topo_slot    = geo.instances().create<Vector4i>(builtin::topo);
    auto mu_slot      = geo.instances().create<Float>("mu");
    auto lambda_slot  = geo.instances().create<Float>("lambda");
    auto min_sep_slot = geo.instances().create<Float>("min_separate_distance");

    auto topo_view    = view(*topo_slot);
    auto mu_view      = view(*mu_slot);
    auto lambda_view  = view(*lambda_slot);
    auto min_sep_view = view(*min_sep_slot);

    for(auto i : range(stitched_vert_tri_ids.size()))
    {
        IndexT          v_id   = stitched_vert_tri_ids[i](0);
        IndexT          tri_id = stitched_vert_tri_ids[i](1);
        const Vector3i& tri    = tri_topo[tri_id];
        topo_view[i]           = Vector4i{v_id, tri(0), tri(1), tri(2)};
        mu_view[i]             = mu;
        lambda_view[i]         = lambda;
        min_sep_view[i]        = min_separate_distance;
    }

    return geo;
}

Json SoftVertexTriangleStitch::default_config()
{
    return Json::object();
}

U64 SoftVertexTriangleStitch::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
