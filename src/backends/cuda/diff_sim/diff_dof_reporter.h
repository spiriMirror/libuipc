#pragma once
#include <sim_system.h>
#include <diff_sim/global_diff_sim_manager.h>

namespace uipc::backend::cuda
{
class DiffDofReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    class BuildInfo
    {
      public:
    };

    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_extent(GlobalDiffSimManager::DiffDofExtentInfo& info) = 0;
    virtual void do_assemble(GlobalDiffSimManager::DiffDofInfo& info) = 0;

  private:
    friend class GlobalDiffSimManager;
    virtual void do_build() override final;
    void         report_extent(GlobalDiffSimManager::DiffDofExtentInfo& info);
    void         assemble_diff_dof(GlobalDiffSimManager::DiffDofInfo& info);
    SizeT        m_index = ~0ull;
};
}  // namespace uipc::backend::cuda