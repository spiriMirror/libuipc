#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;

    uids.push_back(UIDInfo{.uid  = 20,
                           .name = "AffineBodyPrismaticJoint",
                           .type = string{builtin::InterAffineBody}});

    uids.push_back(UIDInfo{.uid  = 21,
                           .name = "AffineBodyDrivingPrismaticJoint",
                           .type = string{builtin::Constraint}});

    return uids;
}
}  // namespace uipc::constitution