#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
template <int N>
auto FiniteElementMethod::FilteredInfo<N>::geo_infos() const noexcept
    -> span<const GeoInfo>
{
    auto info = this->constitution_info();
    return span{m_impl->geo_infos}.subspan(info.geo_info_offset, info.geo_info_count);
}

template <int N>
auto FiniteElementMethod::FilteredInfo<N>::constitution_info() const noexcept
    -> const ConstitutionInfo&
{
    if constexpr(N == 0)
    {
        return m_impl->codim_0d_constitution_infos[m_index_in_dim];
    }
    else if constexpr(N == 1)
    {
        return m_impl->codim_1d_constitution_infos[m_index_in_dim];
    }
    else if constexpr(N == 2)
    {
        return m_impl->codim_2d_constitution_infos[m_index_in_dim];
    }
    else if constexpr(N == 3)
    {
        return m_impl->fem_3d_constitution_infos[m_index_in_dim];
    }
}
template <int N>
size_t FiniteElementMethod::FilteredInfo<N>::vertex_count() const noexcept
{
    return constitution_info().vertex_count;
}
template <int N>
size_t FiniteElementMethod::FilteredInfo<N>::primitive_count() const noexcept
{
    return constitution_info().primitive_count;
}

template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::_for_each(span<const GeoInfo>             geo_infos,
                                    span<S<geometry::GeometrySlot>> geo_slots,
                                    ViewGetter&&                    view_getter,
                                    ForEach&& for_each_action) noexcept
{
    ForEachInfo foreach_info;

    for(auto& geo_info : geo_infos)
    {
        auto geo_slot = geo_slots[geo_info.geo_slot_index];

        auto sc = geo_slot->geometry().template as<geometry::SimplicialComplex>();

        UIPC_ASSERT(sc, "Only simplicial complex is supported");

        auto view = view_getter(*sc);

        foreach_info.m_local_index = 0;

        for(auto&& item : view)
        {
            for_each_action(foreach_info, item);

            ++foreach_info.m_local_index;
            ++foreach_info.m_global_index;
        }
    }
}

template <typename ForEachGeometry>
void FiniteElementMethod::_for_each(span<const GeoInfo>             geo_infos,
                                    span<S<geometry::GeometrySlot>> geo_slots,
                                    ForEachGeometry&& for_every_geometry) noexcept
{
    ForEachInfo foreach_info;
    for(auto& geo_info : geo_infos)
    {
        auto geo_slot = geo_slots[geo_info.geo_slot_index];

        auto sc = geo_slot->geometry().template as<geometry::SimplicialComplex>();

        UIPC_ASSERT(sc, "Only simplicial complex is supported");


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
void FiniteElementMethod::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                   ViewGetter&&                    view_getter,
                                   ForEach&& for_each_action) noexcept
{
    _for_each(span{m_impl.geo_infos},
              geo_slots,
              std::forward<ViewGetter>(view_getter),
              std::forward<ForEach>(for_each_action));
}

template <typename ForEachGeometry>
void FiniteElementMethod::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                   ForEachGeometry&& for_each) noexcept
{
    _for_each(span{m_impl.geo_infos}, geo_slots, std::forward<ForEachGeometry>(for_each));
}

template <int N>
template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::FilteredInfo<N>::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                    ViewGetter&& view_getter,
                                                    ForEach&& for_each_action) const noexcept
{
    FiniteElementMethod::template _for_each(geo_infos(),
                                            geo_slots,
                                            std::forward<ViewGetter>(view_getter),
                                            std::forward<ForEach>(for_each_action));
}
}  // namespace uipc::backend::cuda
