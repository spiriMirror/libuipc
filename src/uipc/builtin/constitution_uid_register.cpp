#include <uipc/builtin/constitution_uid_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
const ConstitutionUIDRegister& ConstitutionUIDRegister::instance() noexcept
{
    static ConstitutionUIDRegister instance;
    return instance;
}
}  // namespace uipc::builtin
