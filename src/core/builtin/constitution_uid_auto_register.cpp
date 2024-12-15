#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
ConstitutionUIDAutoRegister::ConstitutionUIDAutoRegister(Creator creator) noexcept
{
    creators().push_back(creator);
}

auto ConstitutionUIDAutoRegister::creators() noexcept -> list<Creator>&
{
    static list<Creator> creators;
    return creators;
}
}  // namespace uipc::builtin
