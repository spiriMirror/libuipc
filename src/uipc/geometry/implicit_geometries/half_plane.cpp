#include <uipc/builtin/implicit_geometry_uid_auto_register.h>

namespace uipc::geometry
{
REGISTER_IMPLICIT_GEOMETRY_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{
        .uid  = 1ull,
        .name = "HalfPlane",
    });
}
}  // namespace uipc::geometry
