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
template <int N>
template <typename ForEach, typename ViewGetter>
void FiniteElementMethod::FilteredInfo<N>::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                    ViewGetter&& view_getter,
                                                    ForEach&& for_each_action) const noexcept
{
    SizeT local_vertex_offset = 0;

    for(auto& geo_info : geo_infos())
    {
        auto geo_slot = geo_slots[geo_info.geo_slot_index];

        auto sc = geo_slot->geometry().template as<geometry::SimplicialComplex>();

        UIPC_ASSERT(sc, "Only simplicial complex is supported");

        auto view = view_getter(*sc);

        for(auto&& item : view)
        {
            for_each_action(local_vertex_offset++, item);
        }
    }
}
}  // namespace uipc::backend::cuda
