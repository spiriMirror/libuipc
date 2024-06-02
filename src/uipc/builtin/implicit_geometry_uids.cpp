#include <uipc/builtin/implicit_geometry_uid_register.h>

namespace uipc::builtin
{
ImplicitGeometryUIDRegister::ImplicitGeometryUIDRegister()
{
    create(UIDInfo{.uid = 0, .name = "Empty"});

    create(UIDInfo{.uid = 1, .name = "HalfPlane"});
}
}  // namespace uipc::builtin
