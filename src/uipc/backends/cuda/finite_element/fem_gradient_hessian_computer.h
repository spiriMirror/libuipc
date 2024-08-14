#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
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
        FiniteElementMethod*       finite_element_method = nullptr;
        FiniteElementMethod::Impl& fem()
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
