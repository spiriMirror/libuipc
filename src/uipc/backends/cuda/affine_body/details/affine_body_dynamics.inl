#include <uipc/common/zip.h>
#include "affine_body_dynamics.h"

namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                         span<const GeoInfo> geo_infos,
                                         ViewGetterF&&       getter,
                                         ForEachF&&          for_each)
{
    for(auto&& geo_info : geo_infos)
    {
        auto geo_index = geo_info.geo_slot_index;
        auto sc = geo_slots[geo_index]->geometry().as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "Geometry({}) is not a simplicial complex, why can it happen?",
                    sc->type());

        auto  attr_view = getter(*sc);
        SizeT local_i   = 0;
        for(auto&& value : attr_view)
            for_each(local_i++, value);
    }
}

template <typename ForEachGeomatry>
void AffineBodyDynamics::Impl::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                         span<const GeoInfo> geo_infos,
                                         ForEachGeomatry&&   for_every_geometry)
{
    for(auto&& geo_info : geo_infos)
    {
        auto geo_index = geo_info.geo_slot_index;
        auto sc = geo_slots[geo_index]->geometry().as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "Geometry({}) is not a simplicial complex, why can it happen?",
                    sc->type());

        for_every_geometry(*sc);
    }
}

template <typename ForEachGeomatry>
void AffineBodyDynamics::Impl::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                        ForEachGeomatry&& for_every_geometry)
{
    _for_each(geo_slots, geo_infos, std::forward<ForEachGeomatry>(for_every_geometry));
}

template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                        ViewGetterF&& getter,
                                        ForEachF&&    for_each)
{
    _for_each(geo_slots,
              geo_infos,
              std::forward<ViewGetterF>(getter),
              std::forward<ForEachF>(for_each));
}

template <typename T>
muda::BufferView<T> AffineBodyDynamics::Impl::subview(DeviceBuffer<T>& buffer,
                                                      SizeT constitution_index) const noexcept
{
    return buffer.view(constitution_body_offsets[constitution_index],
                       constitution_body_counts[constitution_index]);
}

template <typename T>
span<T> AffineBodyDynamics::Impl::subview(vector<T>& buffer, SizeT constitution_index) const noexcept
{
    return span{buffer}.subspan(constitution_body_offsets[constitution_index],
                                constitution_body_counts[constitution_index]);
}

template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ViewGetterF&& getter,
                                                ForEachF&&    for_each) const
{
    auto constitution_geo_offset = m_impl->constitution_geo_offsets[m_constitution_index];
    auto constitution_geo_count = m_impl->constitution_geo_counts[m_constitution_index];

    auto geo_info_span =
        span{m_impl->geo_infos}.subspan(constitution_geo_offset, constitution_geo_count);

    m_impl->_for_each(geo_slots,
                      geo_info_span,
                      std::forward<ViewGetterF>(getter),
                      std::forward<ForEachF>(for_each));
}
}  // namespace uipc::backend::cuda
