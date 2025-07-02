#include <uipc/constitution/soft_vertex_stitch.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>


namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 22;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;

    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "SoftVertexStitch",
                           .type = string{builtin::InterPrimitive}});

    return uids;
}

SoftVertexStitch::SoftVertexStitch(const Json& config)
{
    m_config = config;
}

geometry::Geometry SoftVertexStitch::create_geometry(const SlotTuple& aim_geo_slots,
                                                     span<const Vector2i> stitched_vert_ids,
                                                     Float kappa_v) const
{
    geometry::Geometry geo;

    auto uids      = geo.meta().create<U64>(builtin::constitution_uid);
    view(*uids)[0] = this->uid();

    auto geo_ids = geo.meta().create<Vector2i>("geo_ids");
    {
        auto&& [l, r] = aim_geo_slots;
        UIPC_ASSERT(l->geometry().instances().size() == 1,
                    "stitch must have exactly one instance, found {} instances",
                    l->geometry().instances().size());
        UIPC_ASSERT(r->geometry().instances().size() == 1,
                    "Link must have exactly one instance, found {} instances",
                    r->geometry().instances().size());
        view(*geo_ids)[0] = Vector2i{l->id(), r->id()};
    }

    geo.instances().resize(stitched_vert_ids.size());

    auto topo      = geo.instances().create<Vector2i>(builtin::topo);
    auto topo_view = view(*topo);
    std::ranges::copy(stitched_vert_ids, topo_view.begin());

    auto kappa      = geo.instances().create<Float>("kappa");
    auto kappa_view = view(*kappa);
    std::ranges::fill(kappa_view, kappa_v);

    return geo;
}

Json SoftVertexStitch::default_config()
{
    return Json::object();
}

U64 SoftVertexStitch::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution