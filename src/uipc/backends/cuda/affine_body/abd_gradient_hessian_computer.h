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
        AffineBodyDynamics*       affine_body_dynamics = nullptr;
        AffineBodyAnimator*       affine_body_animator = nullptr;
        AffineBodyDynamics::Impl& abd() noexcept;

        void compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info);
    };

  private:
    virtual void do_build() override;
    Impl         m_impl;
};
}  // namespace uipc::backend::cuda
