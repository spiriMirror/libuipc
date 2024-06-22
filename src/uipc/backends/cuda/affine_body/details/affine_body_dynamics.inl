#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::for_each_body(span<P<geometry::GeometrySlot>> geo_slots,
                                             ViewGetterF&& getter,
                                             ForEachF&&    for_each)
{
    SizeT I = 0;
    for(auto&& [body_offset, body_count] : zip(h_abd_geo_body_offsets, h_abd_geo_body_counts))
    {
        auto& info      = this->h_body_infos[body_offset];
        auto& sc        = this->geometry(geo_slots, info);
        auto  attr_view = getter(sc);
        for(auto&& value : attr_view)
            for_each(I++, value);
    }
}

template <typename T>
muda::BufferView<T> AffineBodyDynamics::Impl::subview(DeviceBuffer<T>& buffer,
                                                      SizeT constitution_index) const noexcept
{
    return buffer.view(h_constitution_body_offsets[constitution_index],
                       h_constitution_body_counts[constitution_index]);
}

template <typename T>
span<T> AffineBodyDynamics::Impl::subview(vector<T>& buffer, SizeT constitution_index) const noexcept
{
    return span{buffer}.subspan(h_constitution_body_offsets[constitution_index],
                                h_constitution_body_counts[constitution_index]);
}

template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::FilteredInfo::for_each_body(span<P<geometry::GeometrySlot>> geo_slots,
                                                     ViewGetterF&& getter,
                                                     ForEachF&& for_each) const
{
    SizeT I = 0;

    auto constitution_geo_offset = m_impl->h_constitution_geo_offsets[constitution_index];
    auto constitution_geo_count = m_impl->h_constitution_geo_counts[constitution_index];
    auto sub_abd_geo_body_offsets =
        span{m_impl->h_abd_geo_body_offsets}.subspan(constitution_geo_offset,
                                                     constitution_geo_count);
    auto sub_abd_geo_body_counts =
        span{m_impl->h_abd_geo_body_counts}.subspan(constitution_geo_offset,
                                                    constitution_geo_count);

    for(auto&& [body_offset, body_count] : zip(sub_abd_geo_body_offsets, sub_abd_geo_body_counts))
    {
        const auto& info      = m_impl->h_body_infos[body_offset];
        auto&       sc        = this->geometry(geo_slots, info);
        auto        attr_view = getter(sc);
        for(auto&& value : attr_view)
            for_each(I++, value);
    }
}
}  // namespace uipc::backend::cuda
