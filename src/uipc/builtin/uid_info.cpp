#include <uipc/builtin/uid_info.h>

namespace uipc::builtin
{
bool UIDInfo::is_official_builtin_uid(U64 uid) const noexcept
{
    return uid < UserDefinedUIDStart;
}
bool UIDInfo::is_user_defined_uid(U64 uid) const noexcept
{
    return uid >= UserDefinedUIDStart;
}
}  // namespace uipc::builtin