namespace uipc::backend::cuda
{
template <typename ViewGetter, typename ForEach>
void AffineBodyExtraConstitution::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                          ViewGetter&& view_getter,
                                                          ForEach&&    for_each_action)
{
    auto geo_infos = this->geo_infos();
    AffineBodyDynamics::Impl::_for_each(geo_slots,
                                         geo_infos,
                                         std::forward<ViewGetter>(view_getter),
                                         std::forward<ForEach>(for_each_action));
}

template <typename ForEach>
void AffineBodyExtraConstitution::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                          ForEach&& for_each_action)
{
    auto geo_infos = this->geo_infos();
    AffineBodyDynamics::Impl::_for_each(geo_slots, geo_infos, std::forward<ForEach>(for_each_action));
}
}  // namespace uipc::backend::cuda
