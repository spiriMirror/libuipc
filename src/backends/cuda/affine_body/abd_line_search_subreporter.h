#pragma once
#include <sim_system.h>
#include <affine_body/abd_line_search_reporter.h>

namespace uipc::backend::cuda
{
class ABDLineSearchSubreporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    using ExtentInfo = ABDLineSearchReporter::ExtentInfo;
    using EnergyInfo = ABDLineSearchReporter::EnergyInfo;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info)          = 0;
    virtual void do_init(InitInfo& info)            = 0;
    virtual void do_report_extent(ExtentInfo& info) = 0;
    virtual void do_report_energy(EnergyInfo& info) = 0;

  private:
    virtual void do_build() override final;

    friend class ABDLineSearchReporter;
    void init();

    void report_extent(ExtentInfo& info);
    void report_energy(EnergyInfo& info);

    IndexT m_index = -1;
};
}  // namespace uipc::backend::cuda