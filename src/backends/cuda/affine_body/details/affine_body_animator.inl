namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void AffineBodyAnimator::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ViewGetterF&& getter,
                                                ForEachF&&    for_each)
{
    AffineBodyDynamics::Impl::_for_each(geo_slots,
                                        this->anim_geo_infos(),
                                        std::forward<ViewGetterF>(getter),
                                        std::forward<ForEachF>(for_each));
}

template <typename ForEachGeomatry>
void AffineBodyAnimator::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                ForEachGeomatry&& for_every_geometry)
{
    AffineBodyDynamics::Impl::_for_each(geo_slots,
                                        this->anim_geo_infos(),
                                        std::forward<ForEachGeomatry>(for_every_geometry));
}
}  // namespace uipc::backend::cuda