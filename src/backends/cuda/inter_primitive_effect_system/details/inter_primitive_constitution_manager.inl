namespace uipc::backend::cuda
{
template <typename ForEachGeometry>
void InterPrimitiveConstitutionManager::_for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                  span<const InterGeoInfo> geo_infos,
                                                  ForEachGeometry&& for_every_geometry)
{
    ForEachInfo foreach_info;

    for(auto&& geo_info : geo_infos)
    {
        auto  geo_index = geo_info.geo_slot_index;
        auto& geo       = geo_slots[geo_index]->geometry();

        if constexpr(std::is_invocable_v<ForEachGeometry, const ForEachInfo&, geometry::Geometry&>)
        {
            foreach_info.m_geo_info = &geo_info;
            for_every_geometry(foreach_info, geo);
            ++foreach_info.m_index;
        }
        else if constexpr(std::is_invocable_v<ForEachGeometry, geometry::Geometry&>)
        {
            for_every_geometry(geo);
        }
        else
        {
            static_assert("Invalid ForEachGeometry");
        }
    }
}

template <typename ForEachGeometry>
void InterPrimitiveConstitutionManager::FilteredInfo::for_each(
    span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry) const
{
    InterPrimitiveConstitutionManager::_for_each(
        geo_slots, this->inter_geo_infos(), std::forward<ForEachGeometry>(for_every_geometry));
}
}