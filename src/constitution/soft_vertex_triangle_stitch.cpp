#include <uipc/constitution/soft_vertex_triangle_stitch.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/attribute_friend.h>
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
    UIPC_ASSERT_THROW(topo_slot, "pair_geometry must have instances with topo (Vector2i) attribute");
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
    geometry::Geometry geo;

    auto uids      = geo.meta().create<U64>(builtin::constitution_uid);
    view(*uids)[0] = ConstitutionUID;

    auto geo_ids = geo.meta().create<Vector2i>("geo_ids");
    {
        auto&& [l, r] = aim_geo_slots;
        UIPC_ASSERT_THROW(l->geometry().instances().size() == 1,
                    "stitch must have exactly one instance, found {} instances",
                    l->geometry().instances().size());
        UIPC_ASSERT_THROW(r->geometry().instances().size() == 1,
                    "stitch must have exactly one instance, found {} instances",
                    r->geometry().instances().size());
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
