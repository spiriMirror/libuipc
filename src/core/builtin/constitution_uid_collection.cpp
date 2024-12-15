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
        for(auto& uid : uid_infos)
        {
            create(uid);
            UIPC_ASSERT(!uid.type.empty(),
                        "ConstitutionUIDCollection: Constitution type is empty for UID: {}",
                        uid.uid);
        }
    }
}
}  // namespace uipc::builtin
