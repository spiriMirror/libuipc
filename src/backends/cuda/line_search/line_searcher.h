#pragma once
#include <sim_system.h>
#include <optional>

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

    class EnergyInfo
    {
      public:
        EnergyInfo(LineSearcher* impl) noexcept;
        Float dt() noexcept;
        void  energy(Float e) noexcept;
        bool  is_initial() noexcept;


      private:
        friend class LineSearcher;
        LineSearcher*        m_impl       = nullptr;
        std::optional<Float> m_energy     = std::nullopt;
        bool                 m_is_initial = false;
    };

    void add_reporter(LineSearchReporter* reporter);
    void add_reporter(SimSystem&                        system,
                      std::string_view                  energy_name,
                      std::function<void(EnergyInfo)>&& energy_reporter);

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
    SimActionCollection<void(EnergyInfo)>       m_energy_reporters;
    list<std::string>                           m_energy_reporter_names;

    vector<Float>     m_energy_values;
    bool              m_report_energy = false;
    std::stringstream m_report_stream;
    Float             m_dt       = 0.0;
    IndexT            m_max_iter = 64;
};
}  // namespace uipc::backend::cuda
