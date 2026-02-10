#pragma once
#include <line_search/line_search_reporter.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <dytopo_effect_system/dytopo_effect_reporter.h>

namespace uipc::backend::cuda
{
class DyTopoEffectLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class Impl;

    class Impl
    {
      public:
        void init();
        void compute_energy(bool is_init);

        SimSystemSlot<GlobalDyTopoEffectManager> global_dytopo_effect_manager;

        muda::DeviceVar<Float>    energy;
        muda::DeviceBuffer<Float> energies;
        Float                     reserve_ratio = 1.5;

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

    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;

  private:
    virtual void do_init(LineSearchReporter::InitInfo& info) override;
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
