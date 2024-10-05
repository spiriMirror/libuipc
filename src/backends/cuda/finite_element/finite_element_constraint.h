#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_animator.h>
#include <animator/utils.h>

namespace uipc::backend::cuda
{
class FiniteElementConstraint : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                          = 0;
    virtual U64  get_uid() const noexcept                           = 0;
    virtual void do_init(FiniteElementAnimator::FilteredInfo& info) = 0;
    virtual void do_step(FiniteElementAnimator::FilteredInfo& info) = 0;
    virtual void do_report_extent(FiniteElementAnimator::ReportExtentInfo& info) = 0;
    virtual void do_compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info) = 0;

  private:
    friend class FiniteElementAnimator;
    virtual void do_build() override final;

    void init(FiniteElementAnimator::FilteredInfo& info);
    void step(FiniteElementAnimator::FilteredInfo& info);
    void report_extent(FiniteElementAnimator::ReportExtentInfo& info);
    void compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info);
    void compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info);
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
