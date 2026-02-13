#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = 669,
                           .name = "AffineBodyPrismaticJointLimit",
                           .type = string{builtin::InterAffineBody}});
    return uids;
}
}  // namespace uipc::constitution
