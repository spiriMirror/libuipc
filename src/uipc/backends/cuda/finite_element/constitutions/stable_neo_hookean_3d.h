#pragma once
#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
class StableNeoHookean3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 9;

    using FEM3DConstitution::FEM3DConstitution;

  protected:
    virtual U64  get_constitution_uid() const override;
    virtual void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda
