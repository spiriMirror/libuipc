#include <uipc/builtin/implicit_geometry_uid_auto_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
ImplicitGeometryUIDAutoRegister::ImplicitGeometryUIDAutoRegister(Creator creator) noexcept
{
    creators().push_back(creator);
}

auto ImplicitGeometryUIDAutoRegister::creators() noexcept -> list<Creator>&
{
    static list<Creator> creators;
    return creators;
}
}  // namespace uipc::builtin
