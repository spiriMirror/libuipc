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

        muda::DeviceVar<Float>    energy;
        muda::DeviceBuffer<Float> energies;
        SizeT                     reserve_ratio = 1.5;

        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size)
        {
            if(size > buffer.capacity())
            {
                buffer.reserve(size * reserve_ratio);
            }
            buffer.resize(size);
        }
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
