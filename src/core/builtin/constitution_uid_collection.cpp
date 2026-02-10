#include <uipc/builtin/constitution_uid_collection.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/common/log.h>

namespace uipc::builtin
{
const ConstitutionUIDCollection& ConstitutionUIDCollection::instance() noexcept
{
    static ConstitutionUIDCollection instance;
    return instance;
}

ConstitutionUIDCollection::ConstitutionUIDCollection()
{
    auto& creators = ConstitutionUIDAutoRegister::creators();
    for(auto& C : creators)
    {
        list<UIDInfo> uid_infos = C();
        CreatorInfo   creator_info;
        creator_info.file = C.file();
        creator_info.line = C.line();

        for(auto& uid : uid_infos)
        {
            UIPC_ASSERT(!uid.type.empty(),
                        "ConstitutionUIDCollection: Constitution type is empty for UID: {}, creator: {}({})",
                        uid.uid,
                        creator_info.file,
                        creator_info.line);

            create(uid, creator_info);
        }
    }
}
}  // namespace uipc::builtin
