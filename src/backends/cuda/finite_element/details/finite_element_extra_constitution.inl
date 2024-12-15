namespace uipc::backend::cuda
{
template <typename ForEach, typename ViewGetter>
void FiniteElementExtraConstitution::FilteredInfo::for_each(span<S<geometry::GeometrySlot>> geo_slots,
                                                            ViewGetter&& view_getter,
                                                            ForEach&& for_each_action)
{
    auto geo_infos = this->geo_infos();
    FiniteElementMethod::_for_each(geo_infos,
                                   geo_slots,
                                   std::forward<ViewGetter>(view_getter),
                                   std::forward<ForEach>(for_each_action));
}
template <typename ForEach>
void uipc::backend::cuda::FiniteElementExtraConstitution::FilteredInfo::for_each(
    span<S<geometry::GeometrySlot>> geo_slots, ForEach&& for_each_action)
{
    auto geo_infos = this->geo_infos();
    FiniteElementMethod::_for_each(geo_infos, geo_slots, std::forward<ForEach>(for_each_action));
}
}  // namespace uipc::backend::cuda