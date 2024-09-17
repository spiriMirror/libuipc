#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_animator.h>
#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
class FEMGradientHessianComputer final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        void compute_gradient_and_hessian(GradientHessianComputer::ComputeInfo& info);
        SimSystemSlot<FiniteElementMethod>   finite_element_method;
        SimSystemSlot<FiniteElementAnimator> finite_element_animator;
        FiniteElementMethod::Impl&           fem()
        {
            return finite_element_method->m_impl;
        }
    };

  protected:
    virtual void do_build() override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
