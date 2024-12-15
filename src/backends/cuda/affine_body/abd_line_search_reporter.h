#pragma once
#include <sim_system.h>
#include <line_search/line_search_reporter.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_animator.h>

namespace uipc::backend::cuda
{
class ABDLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl
    {
      public:
        void record_start_point(LineSearcher::RecordInfo& info);
        void step_forward(LineSearcher::StepInfo& info);
        void compute_energy(LineSearcher::EnergyInfo& info);

        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
        SimSystemSlot<AffineBodyAnimator> affine_body_animator;

        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }
    };

  protected:
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::EnergyInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda