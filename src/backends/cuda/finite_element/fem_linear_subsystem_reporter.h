#pragma once
#include <sim_system.h>
#include <finite_element/fem_linear_subsystem.h>

namespace uipc::backend::cuda
{
class FEMLinearSubsystemReporter : public SimSystem
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

    using ReportExtentInfo = FEMLinearSubsystem::ReportExtentInfo;
    using AssembleInfo     = FEMLinearSubsystem::AssembleInfo;

  protected:
    virtual void do_build(BuildInfo& info)                = 0;
    virtual void do_init(InitInfo& info)                  = 0;
    virtual void do_report_extent(ReportExtentInfo& info) = 0;
    virtual void do_assemble(AssembleInfo& info)          = 0;

  private:
    friend class FEMLinearSubsystem;
    virtual void do_build() final override;
    void         init();
    void         report_extent(ReportExtentInfo& info);
    void         assemble(AssembleInfo& info);

    IndexT m_index = -1;
};
}  // namespace uipc::backend::cuda
