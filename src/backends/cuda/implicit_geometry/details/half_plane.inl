#pragma once

namespace uipc::backend::cuda
{
template <typename ForEachGeometry>
void HalfPlane::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                          span<GeoInfo>                   geo_infos,
                          ForEachGeometry&&               for_every_geometry)
{
    ForEachInfo foreach_info;

    for(const GeoInfo& geo_info : geo_infos)
    {
        auto geo_index = geo_info.geo_slot_index;
        auto sc = geo_slots[geo_index]->geometry().as<geometry::ImplicitGeometry>();
        UIPC_ASSERT(sc,
                    "Geometry({}) is not a ImplicitGeometry, why can it happen?",
                    sc->type());

        if constexpr(std::is_invocable_v<ForEachGeometry, const ForEachInfo&, geometry::ImplicitGeometry&>)
        {
            foreach_info.m_geo_info = &geo_info;
            for_every_geometry(foreach_info, *sc);
            ++foreach_info.m_global_index;
        }
        else if constexpr(std::is_invocable_v<ForEachGeometry, geometry::ImplicitGeometry&>)
        {
            for_every_geometry(*sc);
        }
        else
        {
            static_assert("Invalid ForEachGeometry");
        }
    }
}
}  // namespace uipc::backend::cuda