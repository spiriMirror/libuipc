#pragma once
#include <sim_system.h>
#include <line_search/line_search_reporter.h>
#include <affine_body/affine_body_dynamics.h>
#include <utils/offset_count_collection.h>


namespace uipc::backend::cuda
{
class ABDLineSearchSubreporter;
class ABDLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class ExtentInfo
    {
      public:
        void energy_count(SizeT count) { m_energy_count = count; }

      private:
        friend class ABDLineSearchReporter;
        SizeT m_energy_count = 0;
    };

    class EnergyInfo
    {
      public:
        muda::BufferView<Float> energies() const { return m_energies; }

      private:
        friend class ABDLineSearchReporter;
        muda::BufferView<Float> m_energies;
    };

    class Impl
    {
      public:
        void init(LineSearchReporter::InitInfo& info);
        void record_start_point(LineSearcher::RecordInfo& info);
        void step_forward(LineSearcher::StepInfo& info);
        void compute_energy(LineSearcher::EnergyInfo& info);

        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;

        SimSystemSlotCollection<ABDLineSearchSubreporter> reporters;
        OffsetCountCollection<IndexT> reporter_energy_offsets_counts;
        muda::DeviceBuffer<Float>     reporter_energies;
        muda::DeviceVar<Float>        total_reporter_energy;

        AffineBodyDynamics::Impl& abd() const
        {
            return affine_body_dynamics->m_impl;
        }
    };

  private:
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_init(LineSearchReporter::InitInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::EnergyInfo& info) override;

    friend class ABDLineSearchSubreporter;
    void add_reporter(ABDLineSearchSubreporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda