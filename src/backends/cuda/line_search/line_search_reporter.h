#pragma once
#include <sim_system.h>
#include <line_search/line_searcher.h>

namespace uipc::backend::cuda
{
class LineSearchReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

  protected:
    virtual void do_init(InitInfo& info)                               = 0;
    virtual void do_record_start_point(LineSearcher::RecordInfo& info) = 0;
    virtual void do_step_forward(LineSearcher::StepInfo& info)         = 0;
    virtual void do_compute_energy(LineSearcher::EnergyInfo& info)     = 0;


    virtual void do_build(BuildInfo& info);

  private:
    friend class LineSearcher;
    virtual void do_build() override final;
    void         record_start_point(LineSearcher::RecordInfo& info);
    void         step_forward(LineSearcher::StepInfo& info);
    void         compute_energy(LineSearcher::EnergyInfo& info);
    void         init();

    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
