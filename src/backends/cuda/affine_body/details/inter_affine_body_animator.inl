namespace uipc::backend::cuda
{
template <typename ForEachGeometry>
void InterAffineBodyAnimator::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                     ForEachGeometry&& for_every_geometry)
{
    InterAffineBodyConstitutionManager::_for_each(
        geo_slots, this->anim_inter_geo_infos(), std::forward<ForEachGeometry>(for_every_geometry));
}

}  // namespace uipc::backend::cuda