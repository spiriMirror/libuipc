#pragma once
#include <sim_system.h>
#include <affine_body/abd_linear_subsystem.h>

namespace uipc::backend::cuda
{
class ABDLinearSubsystemReporter : public SimSystem
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

    using ReportExtentInfo = ABDLinearSubsystem::ReportExtentInfo;
    using AssembleInfo     = ABDLinearSubsystem::AssembleInfo;

  protected:
    virtual void do_build(BuildInfo& info)                = 0;
    virtual void do_init(InitInfo& info)                  = 0;
    virtual void do_report_extent(ReportExtentInfo& info) = 0;
    virtual void do_assemble(AssembleInfo& info)          = 0;

  private:
    friend class ABDLinearSubsystem;
    virtual void do_build() final override;
    void         init();  // only be called by ABDLinearSubsystem
    void         report_extent(ReportExtentInfo& info);
    void         assemble(AssembleInfo& info);

    IndexT m_index = -1;
};
}  // namespace uipc::backend::cuda