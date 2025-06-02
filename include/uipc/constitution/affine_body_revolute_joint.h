#pragma once
#include <uipc/constitution/inter_affine_body_constitution.h>

namespace uipc::constitution
{
class UIPC_CONSTITUTION_API AffineBodyRevoluteJoint final : public InterAffineBodyConstitution
{
  public:
    static Json default_config();

    AffineBodyRevoluteJoint(const Json& config = default_config());
    virtual ~AffineBodyRevoluteJoint();


  private:
    virtual U64 get_uid() const noexcept override;
    Json        m_config;
};
}  // namespace uipc::constitution
