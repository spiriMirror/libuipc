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
        void compute_energy(ComputeEnergyInfo& info);
        void compute_gradient_hessian(ComputeGradientHessianInfo& info);

        vector<Float> h_mus;
        vector<Float> h_lambdas;

        muda::DeviceBuffer<Float> mus;
        muda::DeviceBuffer<Float> lambdas;
    };

  protected:
    virtual U64  get_constitution_uid() const override;
    virtual void do_build(BuildInfo& info) override;
    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) override;

  private:
    Impl         m_impl;
    virtual void do_compute_energy(ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override;
};
}  // namespace uipc::backend::cuda
