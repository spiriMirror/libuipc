#include <uipc/world/constitution_uid.h>
#include <uipc/builtin/constitution_uid_register.h>
#include <uipc/common/log.h>

namespace uipc::world
{
ConstitutionUID::ConstitutionUID(U64 uid) noexcept
    : m_uid(uid)
{
    UIPC_ASSERT(builtin::ConstitutionUIDRegister::is_user_defined_uid(uid)
                    || builtin::ConstitutionUIDRegister::instance().exists(uid),
                "ConstitutionUID: Invalid UID: {}, can't find it in official registeration.",
                uid);
}

ConstitutionUID::operator U64() const
{
    return m_uid;
}
}  // namespace uipc::world
