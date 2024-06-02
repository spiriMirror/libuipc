#include <uipc/builtin/constitution_uid_register.h>

namespace uipc::builtin
{
ConstitutionUIDRegister::ConstitutionUIDRegister()
{
    create(UIDInfo{.uid = 0, .name = "Empty"});

    // create 8 AffineBody constitution uids
    for(U64 i = 1; i <= 8; ++i)
        create(UIDInfo{.uid = i, .name = "AffineBody"});
}
}  // namespace uipc::builtin
