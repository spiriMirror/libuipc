#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class AffineBodyConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    U64 constitution_uid() const;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual U64  get_constitution_uid() const                      = 0;
    virtual void do_build(BuildInfo& info)                         = 0;
    virtual void do_filter(AffineBodyDynamics::FilteredInfo& info) = 0;
    virtual void do_compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info) = 0;

  private:
    friend class AffineBodyDynamics;
    friend class ABDLineSearchReporter;

    virtual void do_build() override final;
    void         filter(AffineBodyDynamics::FilteredInfo& info);
    void         compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info);
    void compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info);
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
