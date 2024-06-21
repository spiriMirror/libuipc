#pragma once
#include <sim_system.h>
#include <functional>
#include <uipc/common/list.h>
namespace uipc::backend::cuda
{
class LineSearcher : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class StepInfo
    {
      public:
        Float alpha;
    };

    class ComputeEnergyInfo
    {
      public:
    };

    void on_record_current_state(std::function<void()>&& action);
    void on_step_forward(std::function<void(const StepInfo& info)>&& action);
    void on_compute_energy(std::string_view name,
                           std::function<Float(const ComputeEnergyInfo& info)>&& action);

  protected:
    void build() override;

  private:
    friend class SimEngine;
    void  init();
    void  record_current_state();     // only be called by SimEngine
    void  step_forward(Float alpha);  // only be called by SimEngine
    Float compute_energy();           // only be called by SimEngine

    list<std::function<void()>>                     m_record_current_state;
    list<std::function<void(const StepInfo& info)>> m_step_forwards;
    list<std::function<Float(const ComputeEnergyInfo& info)>> m_compute_energy;
    vector<std::string>                                       m_energy_names;
    vector<Float>                                             m_energy_values;
    bool              m_report_energy = false;
    std::stringstream report_stream;
};
}  // namespace uipc::backend::cuda
