#include <uipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/builtin/implicit_geometry_uid_auto_register.h>
#include <uipc/builtin/geometry_type.h>
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
    auto& creators = ImplicitGeometryUIDAutoRegister::creators();
    for(auto& C : creators)
    {
        list<UIDInfo> uid_infos = C();
        CreatorInfo   creator_info;
        creator_info.file = C.file();
        creator_info.line = C.line();

        for(auto& uid : uid_infos)
        {
            UIPC_ASSERT(!uid.type.empty(),
                        "ImplicitGeometryUIDCollection: ImplicitGeometry type is empty for UID: {}, creator: {}({})",
                        uid.uid,
                        creator_info.file,
                        creator_info.line);

            create(uid, creator_info);
        }
    }
}
}  // namespace uipc::builtin
