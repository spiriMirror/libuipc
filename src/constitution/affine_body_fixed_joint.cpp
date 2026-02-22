#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>

namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 25;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyFixedJoint",
                           .type = string{InterAffineBody}});
    return uids;
}
}  // namespace uipc::constitution
