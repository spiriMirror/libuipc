#pragma once
#include <sim_system.h>
#include <line_search/line_searcher.h>

namespace uipc::backend::cuda
{
class LineSearchReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_record_start_point(LineSearcher::RecordInfo& info)    = 0;
    virtual void do_step_forward(LineSearcher::StepInfo& info)            = 0;
    virtual void do_compute_energy(LineSearcher::ComputeEnergyInfo& info) = 0;
    virtual std::string_view get_name() const noexcept;

  private:
    friend class LineSearcher;
    void             record_start_point(LineSearcher::RecordInfo& info);
    void             step_forward(LineSearcher::StepInfo& info);
    void             compute_energy(LineSearcher::ComputeEnergyInfo& info);
    std::string_view name() const noexcept;
    SizeT            m_index = ~0ull;
};

}  // namespace uipc::backend::cuda
