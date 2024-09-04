#pragma once
#include <affine_body/affine_body_dynamics.h>
#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
class ABDGradientHessianComputer final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        AffineBodyDynamics*       affine_body_dynamics = nullptr;
        AffineBodyDynamics::Impl& abd() noexcept;

        void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);
    };

  private:
    virtual void do_build() override;
    Impl         m_impl;
};
}  // namespace uipc::backend::cuda
