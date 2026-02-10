#pragma once
#include <sim_system.h>
#include <affine_body/abd_line_search_reporter.h>

namespace uipc::backend::cuda
{
class ABDLineSearchSubreporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    using ReportExtentInfo  = ABDLineSearchReporter::ReportExtentInfo;
    using ComputeEnergyInfo = ABDLineSearchReporter::ComputeEnergyInfo;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info)                 = 0;
    virtual void do_init(InitInfo& info)                   = 0;
    virtual void do_report_extent(ReportExtentInfo& info)  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;

  private:
    virtual void do_build() override final;

    friend class ABDLineSearchReporter;
    void init();

    void report_extent(ReportExtentInfo& info);
    void compute_energy(ComputeEnergyInfo& info);

    IndexT m_index = -1;
};
}  // namespace uipc::backend::cuda