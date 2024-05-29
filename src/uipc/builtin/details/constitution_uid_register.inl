namespace fmt
{
// format for fmt::format
template <>
struct formatter<uipc::builtin::ConstitutionUIDInfo>
{
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const uipc::builtin::ConstitutionUIDInfo& info, FormatContext& ctx)
    {
        return format_to(ctx.out(),
                         R"(UID: {}
Name: {}
Author: {}
Email: {}
Website: {}
Description: {})",
                         info.uid,
                         info.name,
                         info.author,
                         info.email,
                         info.website,
                         info.description);
    }
};
}  // namespace fmt