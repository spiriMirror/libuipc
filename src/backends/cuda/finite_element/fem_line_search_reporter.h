#pragma once
#include <sim_system.h>
#include <line_search/line_search_reporter.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_animator.h>

namespace uipc::backend::cuda
{
class FEMLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl
    {
      public:
        void record_start_point(LineSearcher::RecordInfo& info);
        void step_forward(LineSearcher::StepInfo& info);
        void compute_energy(LineSearcher::EnergyInfo& info);

        SimSystemSlot<FiniteElementMethod>   finite_element_method;
        SimSystemSlot<FiniteElementAnimator> finite_element_animator;
        FiniteElementMethod::Impl&           fem()
        {
            return finite_element_method->m_impl;
        }
    };

  protected:
    virtual void do_init(InitInfo& info) override;
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::EnergyInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda