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

Json UIDInfo::to_json() const noexcept
{
    Json j;
    j["uid"]         = uid;
    j["name"]        = name;
    j["type"]        = type;
    j["author"]      = author;
    j["email"]       = email;
    j["website"]     = website;
    j["description"] = description;
    j["extras"]      = extras;
    return j;
}
}  // namespace uipc::builtin