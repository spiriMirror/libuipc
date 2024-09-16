#include <uipc/common/zip.h>

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
    auto& constitution_info = constitution_infos[constitution_index];
    return buffer.view(constitution_info.body_offset, constitution_info.body_count);
}

template <typename T>
span<T> AffineBodyDynamics::Impl::subview(vector<T>& buffer, SizeT constitution_index) const noexcept
{
    auto& constitution_info = constitution_infos[constitution_index];
    return span{buffer}.subspan(constitution_info.body_offset, constitution_info.body_count);
}

template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ViewGetterF&& getter,
                                                ForEachF&&    for_each) const
{
    m_impl->_for_each(geo_slots,
                      geo_infos(),
                      std::forward<ViewGetterF>(getter),
                      std::forward<ForEachF>(for_each));
}

template <typename ForEachGeomatry>
void AffineBodyDynamics::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ForEachGeomatry&& for_every_geometry) const
{
    m_impl->_for_each(geo_slots, geo_infos(), std::forward<ForEachGeomatry>(for_every_geometry));
}
}  // namespace uipc::backend::cuda
