#include <uipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/builtin/implicit_geometry_uid_auto_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
const ImplicitGeometryUIDCollection& ImplicitGeometryUIDCollection::instance() noexcept
{
    static ImplicitGeometryUIDCollection instance;
    return instance;
}

ImplicitGeometryUIDCollection::ImplicitGeometryUIDCollection()
{
    create(UIDInfo{.uid = 0, .name = "Empty"});

    auto& creators = ImplicitGeometryUIDAutoRegister::creators();
    for(auto& C : creators)
    {
        list<UIDInfo> uids = C();
        for(auto& uid : uids)
        {
            create(uid);
        }
    }
}
}  // namespace uipc::builtin
