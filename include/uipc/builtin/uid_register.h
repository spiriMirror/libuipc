#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/builtin/uid_info.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/list.h>

namespace uipc::builtin::details
{
class UIPC_CORE_API UIDRegister
{
  public:
    const UIDInfo& find(U64 uid) const;
    bool           exists(U64 uid) const;
    Json           to_json() const noexcept;

  private:
    unordered_map<U64, UIDInfo> m_uid_to_info;

  protected:
    class CreatorInfo
    {
      public:
        std::string_view file = "no-file-info";
        int              line = -1;
    };

    void create(const UIDInfo& info, const CreatorInfo& creator);
};
}  // namespace uipc::builtin::details
