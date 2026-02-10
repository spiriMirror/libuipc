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

UIDInfoCreator::UIDInfoCreator(std::function<list<UIDInfo>()> creator,
                               std::string_view               file,
                               int                            line) noexcept
    : m_creator{std::move(creator)}
    , m_file{file}
    , m_line{line}
{
}

list<UIDInfo> UIDInfoCreator::operator()() const
{
    return m_creator();
}

std::string_view UIDInfoCreator::file() const noexcept
{
    return m_file;
}

int UIDInfoCreator::line() const noexcept
{
    return m_line;
}
}  // namespace uipc::builtin