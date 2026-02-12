#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = 700,
                           .name = "AffineBodyRevoluteJointLimit",
                           .type = string{builtin::Constraint}});
    return uids;
}
}  // namespace uipc::constitution
