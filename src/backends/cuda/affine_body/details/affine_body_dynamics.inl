#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void AffineBodyDynamics::Impl::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                         span<const GeoInfo> geo_infos,
                                         ViewGetterF&&       getter,
                                         ForEachF&&          for_each)
{
    ForEachInfo foreach_info;

    for(auto&& geo_info : geo_infos)
    {
        auto geo_index = geo_info.geo_slot_index;
        auto sc = geo_slots[geo_index]->geometry().as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "Geometry({}) is not a simplicial complex, why can it happen?",
                    sc->type());

        auto attr_view             = getter(*sc);
        foreach_info.m_local_index = 0;
        for(auto&& value : attr_view)
        {
            for_each(foreach_info, value);

            ++foreach_info.m_local_index;
            ++foreach_info.m_global_index;
        }
    }
}

template <typename ForEachGeometry>
void AffineBodyDynamics::Impl::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                         span<const GeoInfo> geo_infos,
                                         ForEachGeometry&&   for_every_geometry)
{
    ForEachInfo foreach_info;

    for(auto&& geo_info : geo_infos)
    {
        auto geo_index = geo_info.geo_slot_index;
        auto sc = geo_slots[geo_index]->geometry().as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "Geometry({}) is not a simplicial complex, why can it happen?",
                    sc->type());

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

template <typename ForEachGeometry>
void AffineBodyDynamics::Impl::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                        ForEachGeometry&& for_every_geometry)
{
    _for_each(geo_slots, geo_infos, std::forward<ForEachGeometry>(for_every_geometry));
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

template <typename ForEachGeometry>
void AffineBodyDynamics::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ForEachGeometry&& for_every_geometry) const
{
    m_impl->_for_each(geo_slots, geo_infos(), std::forward<ForEachGeometry>(for_every_geometry));
}

template <typename ForEachGeometry>
void AffineBodyDynamics::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                  ForEachGeometry&& for_every_geometry)
{
    m_impl.for_each(geo_slots, std::forward<ForEachGeometry>(for_every_geometry));
}
}  // namespace uipc::backend::cuda
