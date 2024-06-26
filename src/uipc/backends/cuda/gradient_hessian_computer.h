#pragma once
#include <sim_system.h>
#include <functional>
#include <uipc/common/list.h>
namespace uipc::backend::cuda
{
class GradientHessianComputer : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class ComputeInfo
    {
      public:
        ComputeInfo(GradientHessianComputer* impl)
            : m_impl(impl)
        {
        }

        Float dt() const noexcept;

      private:
        GradientHessianComputer* m_impl;
    };
    void on_compute_gradient_hessian(std::function<void(ComputeInfo&)>&& action);

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    void                                    init();
    void                                    compute_gradient_hessian();
    list<std::function<void(ComputeInfo&)>> m_on_compute_gradient_hessian;
    Float                                   m_dt = 0.0;
};
}  // namespace uipc::backend::cuda