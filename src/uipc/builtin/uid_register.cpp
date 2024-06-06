#include <uipc/builtin/uid_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin::details
{
void UIDRegister::create(const UIDInfo& info)
{
    auto it = m_uid_to_info.find(info.uid);
    UIPC_ASSERT(it == m_uid_to_info.end(),
                "UID {} already exists, name={}",
                it->second.uid,
                it->second.name);
    m_uid_to_info[info.uid] = info;
}

const UIDInfo& UIDRegister::find(U64 uid) const
{
    auto it = m_uid_to_info.find(uid);
    UIPC_ASSERT(it != m_uid_to_info.end(), "UID {} not found!", uid);
    return it->second;
}

bool UIDRegister::exists(U64 uid) const
{
    return m_uid_to_info.find(uid) != m_uid_to_info.end();
}
}  // namespace uipc::builtin::details