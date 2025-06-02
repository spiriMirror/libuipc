#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/constitution_type.h>
namespace uipc::constitution
{
static constexpr U64 ConstitutionUID = 18;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = ConstitutionUID,
                           .name = "AffineBodyRevoluteJoint",
                           .type = string{builtin::AffineBody}});
    return uids;
}

Json AffineBodyRevoluteJoint::default_config()
{
    return Json::object();
}

AffineBodyRevoluteJoint::AffineBodyRevoluteJoint(const Json& config)
{
    m_config = config;
}

AffineBodyRevoluteJoint::~AffineBodyRevoluteJoint() = default;

U64 AffineBodyRevoluteJoint::get_uid() const noexcept
{
    return ConstitutionUID;
}
}  // namespace uipc::constitution
