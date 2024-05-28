#include <uipc/builtin/constitution_uid_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
const ConstitutionUIDRegister& ConstitutionUIDRegister::instance() noexcept
{
    static ConstitutionUIDRegister instance;
    return instance;
}

void details::ConstitutionUIDRegister::create(const ConstitutionUIDInfo& info)
{
    auto it = m_uid_to_info.find(info.uid);
    UIPC_ASSERT(it == m_uid_to_info.end(),
                "UID {} already exists, name={}",
                it->second.uid,
                it->second.name);
    m_uid_to_info[info.uid] = info;
}

const ConstitutionUIDInfo& details::ConstitutionUIDRegister::find(U64 uid) const
{
    auto it = m_uid_to_info.find(uid);
    UIPC_ASSERT(it != m_uid_to_info.end(), "UID {} not found!", uid);
    return it->second;
}

bool details::ConstitutionUIDRegister::exists(U64 uid) const
{
    return m_uid_to_info.find(uid) != m_uid_to_info.end();
}
}  // namespace uipc::builtin
