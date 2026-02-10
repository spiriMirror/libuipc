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
                                                     Float kappa_v,
                                                     Float rest_length_v) const
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

    auto rest_length      = geo.instances().create<Float>("rest_length");
    auto rest_length_view = view(*rest_length);
    std::ranges::fill(rest_length_view, rest_length_v);

    return geo;
}

template <typename T>
static auto tuple2array(const std::tuple<T, T>& t)
{
    return std::array<T, 2>{std::get<0>(t), std::get<1>(t)};
}

static auto find_meta_ce(const geometry::Geometry& geo)
{
    auto meta_ce = geo.meta().find<IndexT>(builtin::contact_element_id);

    if(meta_ce)
        return meta_ce->view()[0];
    else
        return 0;
}

static auto find_or_create_vert_ce(geometry::SimplicialComplex& geo)
{
    auto vert_ce = geo.vertices().find<IndexT>(builtin::contact_element_id);

    if(!vert_ce)
    {
        // if the geometry does not have contact element id attribute
        // create default one, and fill with meta contact element id
        IndexT meta_ce_v = find_meta_ce(geo);

        vert_ce = geo.vertices().create<IndexT>(builtin::contact_element_id, 0);
        std::ranges::fill(view(*vert_ce), meta_ce_v);
    }

    return vert_ce;
}

geometry::Geometry SoftVertexStitch::create_geometry(const SlotTuple& aim_geo_slots,
                                                     span<const Vector2i> stitched_vert_ids,
                                                     const ContactElementTuple& contact_elements,
                                                     Float kappa,
                                                     Float rest_length) const
{
    auto geo = create_geometry(aim_geo_slots, stitched_vert_ids, kappa, rest_length);

    auto slots = tuple2array(aim_geo_slots);
    auto ces   = tuple2array(contact_elements);

    for(auto&& [i, slot] : enumerate(slots))
    {
        auto& this_geo = slot->geometry();

        IndexT meta_ce_v    = find_meta_ce(this_geo);
        auto   vert_ce      = find_or_create_vert_ce(this_geo);
        auto   vert_ce_view = view(*vert_ce);

        auto& this_ce = ces[i];


        for(auto&& v_id_pair : stitched_vert_ids)
        {
            auto v_id = v_id_pair[i];

            UIPC_ASSERT(v_id >= 0 && v_id < this_geo.vertices().size(),
                        "Stitched vertex id {} out of range [0, {}) in Geometry({}). Please check the stitched vertex ids and the geometry slots.\n",
                        v_id,
                        this_geo.vertices().size(),
                        slot->id());

            vert_ce_view[v_id] = this_ce.id();
        }
    }

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