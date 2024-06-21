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
    };
    void on_compute_gradient_hessian(std::function<void(const ComputeInfo&)>&& action);

  private:
    friend class SimEngine;
    void                                          compute_gradient_hessian();
    list<std::function<void(const ComputeInfo&)>> m_on_compute_gradient_hessian;
};
}  // namespace uipc::backend::cuda
