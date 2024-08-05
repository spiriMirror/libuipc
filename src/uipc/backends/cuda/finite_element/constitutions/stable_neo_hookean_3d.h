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

    class Impl;

    class Impl
    {
      public:
        void retrieve(WorldVisitor& world, FiniteElementMethod::FEM3DFilteredInfo& info);

        vector<Float> h_mu;
        vector<Float> h_lambda;
    };

  protected:
    virtual U64  get_constitution_uid() const override;
    virtual void do_build(BuildInfo& info) override;

    // Inherited via FEM3DConstitution
    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
