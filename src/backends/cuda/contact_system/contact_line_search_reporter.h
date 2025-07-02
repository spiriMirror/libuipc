#pragma once
#include <line_search/line_search_reporter.h>
#include <contact_system/global_contact_manager.h>
#include <contact_system/contact_reporter.h>

namespace uipc::backend::cuda
{
class GlobalContactManager;
class ContactLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl;

    class Impl
    {
      public:
        void init();
        void do_compute_energy(LineSearcher::EnergyInfo& info);

        SimSystemSlot<GlobalContactManager> global_contact_manager;
        muda::DeviceBuffer<Float>           contact_energies;
        vector<Float>                       h_contact_energies;
    };

  private:
    virtual void do_init(LineSearchReporter::InitInfo& info) override;
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::EnergyInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
