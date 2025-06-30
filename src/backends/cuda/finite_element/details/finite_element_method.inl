#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
template <typename ForEachGeometry>
void FiniteElementMethod::_for_each(span<const GeoInfo>             geo_infos,
                                    span<S<geometry::GeometrySlot>> geo_slots,
                                    ForEachGeometry&& for_every_geometry)
{
    ForEachInfo foreach_info;
    for(auto& geo_info : geo_infos)
    {
        auto geo_slot = geo_slots[geo_info.geo_slot_index];

        auto sc = geo_slot->geometry().template as<geometry::SimplicialComplex>();

        UIPC_ASSERT(sc, "Only simplicial complex is supported");

        foreach_info.m_geo_info = &geo_info;

        if constexpr(std::is_invocable_v<ForEachGeometry, const ForEachInfo&, geometry::SimplicialComplex&>)
        {
            for_every_geometry(foreach_info, *sc);
            ++foreach_info.m_global_index;
        }
        else if constexpr(std::is_invocable_v<ForEachGeometry, geometry::SimplicialComplex&>)
        {
            for_every_geometry(*sc);
        }
        else
        {
            static_assert("Invalid ForEachGeometry");
        }
    }
}

template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::_for_each(span<const GeoInfo>             geo_infos,
                                    span<S<geometry::GeometrySlot>> geo_slots,
                                    ViewGetter&&                    view_getter,
                                    ForEach&& for_each_action)
{
    ForEachInfo foreach_info;

    for(auto& geo_info : geo_infos)
    {
        auto geo_slot = geo_slots[geo_info.geo_slot_index];

        auto sc = geo_slot->geometry().template as<geometry::SimplicialComplex>();

        UIPC_ASSERT(sc, "Only simplicial complex is supported");

        auto view = view_getter(*sc);

        foreach_info.m_local_index = 0;
        foreach_info.m_geo_info    = &geo_info;

        for(auto&& item : view)
        {
            for_each_action(foreach_info, item);

            ++foreach_info.m_local_index;
            ++foreach_info.m_global_index;
        }
    }
}

template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                   ViewGetter&&                    view_getter,
                                   ForEach&& for_each_action)
{
    _for_each(span{m_impl.geo_infos},
              geo_slots,
              std::forward<ViewGetter>(view_getter),
              std::forward<ForEach>(for_each_action));
}

template <typename ForEachGeometry>
void FiniteElementMethod::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                   ForEachGeometry&&               for_each)
{
    _for_each(span{m_impl.geo_infos}, geo_slots, std::forward<ForEachGeometry>(for_each));
}

template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                 ViewGetter&& view_getter,
                                                 ForEach&& for_each_action) const
{
    FiniteElementMethod::template _for_each(geo_infos(),
                                            geo_slots,
                                            std::forward<ViewGetter>(view_getter),
                                            std::forward<ForEach>(for_each_action));
}
}  // namespace uipc::backend::cuda
