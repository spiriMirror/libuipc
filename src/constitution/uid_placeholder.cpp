#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;

    uids.push_back(UIDInfo{.uid  = 21,
                           .name = "AffineBodyDrivingPrismaticJoint",
                           .type = string{builtin::Constraint}});

    uids.push_back(UIDInfo{.uid  = 25,
                           .name = "AffineBodyFixedJoint",
                           .type = string{builtin::InterAffineBody}});

    uids.push_back(UIDInfo{.uid  = 26,
                           .name = "AffineBodySphericalJoint",
                           .type = string{builtin::InterAffineBody}});

    uids.push_back(UIDInfo{.uid  = 27,
                           .name = "AffineBodyDrivingSphericalJoint",
                           .type = string{builtin::Constraint}});

    uids.push_back(UIDInfo{.uid  = 28,
                           .name = "AffineBodyD6Joint",
                           .type = string{builtin::InterAffineBody}});

    uids.push_back(UIDInfo{.uid  = 667,
                           .name = "AffineBodyPrismaticJointExternalForce",
                           .type = string{builtin::Constraint}});

    uids.push_back(UIDInfo{.uid  = 668,
                           .name = "AffineBodyRevoluteJointExternalForce",
                           .type = string{builtin::Constraint}});

    return uids;
}
}  // namespace uipc::constitution