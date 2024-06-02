#pragma once
#include <uipc/common/type_define.h>
#include <string>

namespace uipc::builtin
{
struct UIDInfo
{
    static constexpr U64 OfficialBuiltinUIDStart = 0;
    static constexpr U64 UserDefinedUIDStart     = 1ull << 32;

    U64         uid;
    std::string name;
    std::string author;
    std::string email;
    std::string website;
    std::string description;

    bool is_official_builtin_uid(U64 uid) const noexcept;

    bool is_user_defined_uid(U64 uid) const noexcept;
};
}  // namespace uipc::builtin
