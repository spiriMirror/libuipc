#include <uipc/builtin/implicit_geometry_uid_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
const ImplicitGeometryUIDRegister& ImplicitGeometryUIDRegister::instance() noexcept
{
    static ImplicitGeometryUIDRegister instance;
    return instance;
}
}  // namespace uipc::builtin
