#pragma once
#include <sim_system.h>
#include <cuda_tool/buffer.h>

namespace uipc::backend::cuda
{
class LineSearchReporter;

class LineSearcher : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class RecordInfo
    {
      public:
    };

    class StepInfo
    {
      public:
        Float alpha;
    };

    class ComputeEnergyInfo
    {
      public:
        ComputeEnergyInfo(LineSearcher* impl, cuda_tool::VarView<Float> energy) noexcept;
        Float                     dt() noexcept;
        cuda_tool::VarView<Float> energy() noexcept;
        bool                      is_initial() noexcept;


      private:
        friend class LineSearcher;
        LineSearcher*             m_impl = nullptr;
        cuda_tool::VarView<Float> m_energy;
        bool                      m_energy_set = false;
        bool                      m_is_initial = false;
    };

    void add_reporter(LineSearchReporter* reporter);

    SizeT max_iter() const noexcept;

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    void  init();                           // only be called by SimEngine
    void  record_start_point();             // only be called by SimEngine
    void  step_forward(Float alpha);        // only be called by SimEngine
    Float compute_energy(bool is_initial);  // only be called by SimEngine

    SimSystemSlotCollection<LineSearchReporter> m_reporters;

    cuda_tool::DeviceBuffer<Float>          m_device_energy_values;
    vector<Float>                           m_energy_values;
    bool                                    m_report_energy = false;
    std::stringstream                       m_report_stream;
    S<const geometry::AttributeSlot<Float>> m_dt_attr;
    IndexT                                  m_max_iter = 64;
};
}  // namespace uipc::backend::cuda
