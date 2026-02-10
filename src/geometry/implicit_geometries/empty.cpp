#include <uipc/builtin/implicit_geometry_uid_auto_register.h>
#include <uipc/builtin/geometry_type.h>
namespace uipc::geometry
{
REGISTER_IMPLICIT_GEOMETRY_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid = 0, .name = "Empty", .type = std::string{builtin::ImplicitGeometry}});
    return uids;
}
}  // namespace uipc::geometry
