#pragma once
#include <sim_system.h>
#include <line_search/line_search_reporter.h>
#include <finite_element/finite_element_method.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic;
class FEMLineSearchSubreporter;
class FEMLineSearchReporter final : public LineSearchReporter
{
  public:
    using LineSearchReporter::LineSearchReporter;

    class ReportExtentInfo
    {
      public:
        void energy_count(SizeT count) { m_energy_count = count; }

      private:
        friend class FEMLineSearchReporter;
        SizeT m_energy_count  = 0;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(muda::BufferView<Float> energies, Float dt)
            : m_energies(energies)
            , m_dt(dt)
        {
        }

        muda::BufferView<Float> energies() const { return m_energies; }
        Float                   dt() const noexcept { return m_dt; }

      private:
        friend class FEMLineSearchReporter;
        muda::BufferView<Float> m_energies;
        Float                   m_dt = 0.0;
    };

    class Impl
    {
      public:
        void init(LineSearchReporter::InitInfo& info);
        void record_start_point(LineSearcher::RecordInfo& info);
        void step_forward(LineSearcher::StepInfo& info);
        void compute_energy(LineSearcher::ComputeEnergyInfo& info);

        SimSystemSlot<FiniteElementMethod> finite_element_method;

        SimSystemSlot<FiniteElementKinetic> finite_element_kinetic;
        muda::DeviceBuffer<Float>           kinetic_energies;
        muda::DeviceVar<Float>              total_kinetic_energy;

        SimSystemSlotCollection<FEMLineSearchSubreporter> reporters;
        OffsetCountCollection<IndexT> reporter_energy_offsets_counts;
        muda::DeviceBuffer<Float>     reporter_energies;
        muda::DeviceVar<Float>        total_reporter_energy;

        FiniteElementMethod::Impl& fem()
        {
            return finite_element_method->m_impl;
        }
    };

  protected:
    virtual void do_init(InitInfo& info) override;
    virtual void do_build(LineSearchReporter::BuildInfo& info) override;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) override;
    virtual void do_step_forward(LineSearcher::StepInfo& info) override;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) override;

  private:
    Impl m_impl;

    friend class FEMLineSearchSubreporter;
    void add_reporter(FEMLineSearchSubreporter* reporter);

    friend class FiniteElementKinetic;
    void add_kinetic(FiniteElementKinetic* kinetic);
};
}  // namespace uipc::backend::cuda