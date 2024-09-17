#pragma once
#include <gradient_hessian_computer.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_animator.h>
namespace uipc::backend::cuda
{
class ABDGradientHessianComputer final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
        SimSystemSlot<AffineBodyAnimator> affine_body_animator;
        AffineBodyDynamics::Impl&         abd() noexcept;

        void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);
    };

  private:
    virtual void do_build() override;
    Impl         m_impl;
};
}  // namespace uipc::backend::cuda
