#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                         span<SizeT>   abd_geo_body_offsets,
                                         ViewGetterF&& getter,
                                         ForEachF&&    for_each)
{
    for(auto&& body_offset : abd_geo_body_offsets)
    {
        auto& info      = this->h_body_infos[body_offset];
        auto& sc        = this->geometry(geo_slots, info);
        auto  attr_view = getter(sc);
        SizeT local_i   = 0;
        for(auto&& value : attr_view)
            for_each(local_i++, value);
    }
}

template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                        ViewGetterF&& getter,
                                        ForEachF&&    for_each)
{
    _for_each(geo_slots,
              h_abd_geo_body_offsets,
              std::forward<ViewGetterF>(getter),
              std::forward<ForEachF>(for_each));
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
void AffineBodyDynamics::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ViewGetterF&& getter,
                                                ForEachF&&    for_each) const
{
    auto constitution_geo_offset = m_impl->h_constitution_geo_offsets[m_constitution_index];
    auto constitution_geo_count = m_impl->h_constitution_geo_counts[m_constitution_index];
    auto sub_abd_geo_body_offsets =
        span{m_impl->h_abd_geo_body_offsets}.subspan(constitution_geo_offset,
                                                     constitution_geo_count);

    m_impl->_for_each(geo_slots,
                      sub_abd_geo_body_offsets,
                      std::forward<ViewGetterF>(getter),
                      std::forward<ForEachF>(for_each));
}
}  // namespace uipc::backend::cuda
