#pragma once
#include <line_search/line_search_reporter.h>

namespace uipc::backend::cuda
{
class GlobalContactManager;
class ContactLineSearchReporter : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl;

    class Impl
    {
      public:
        GlobalContactManager* global_contact_manager = nullptr;
    };

  protected:
    void do_build() override;
    void do_record_start_point(LineSearcher::RecordInfo& info) override;
    void do_step_forward(LineSearcher::StepInfo& info) override;
    void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
