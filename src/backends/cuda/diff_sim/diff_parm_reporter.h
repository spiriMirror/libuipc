#pragma once
#include <sim_system.h>
#include <diff_sim/global_diff_sim_manager.h>

namespace uipc::backend::cuda
{
class DiffParmReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;


  protected:
    class BuildInfo
    {
      public:
    };

    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_extent(GlobalDiffSimManager::DiffParmExtentInfo& info) = 0;
    virtual void do_assemble(GlobalDiffSimManager::DiffParmInfo& info) = 0;

  private:
    friend class GlobalDiffSimManager;
    virtual void do_build() override final;
    void         report_extent(GlobalDiffSimManager::DiffParmExtentInfo& info);
    void         assemble(GlobalDiffSimManager::DiffParmInfo& info);
    SizeT        m_index = ~0ull;
};
}  // namespace uipc::backend::cuda