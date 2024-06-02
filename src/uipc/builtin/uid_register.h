#pragma once

#include <uipc/builtin/uid_info.h>
#include <uipc/common/unordered_map.h>

namespace uipc::builtin::details
{
class UIDRegister
{
  public:
    const UIDInfo& find(U64 uid) const;
    bool           exists(U64 uid) const;

  private:
    unordered_map<U64, UIDInfo> m_uid_to_info;

  protected:
    void create(const UIDInfo& info);
};
}  // namespace uipc::builtin::details
