#pragma once
#include <sim_system.h>
#include <contact_system/global_contact_manager.h>
namespace uipc::backend::cuda
{
class ContactReporter : public SimSystem
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
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo&);
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) = 0;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info)      = 0;
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) = 0;

  private:
    friend class GlobalContactManager;
    friend class ContactLineSearchReporter;
    void  init();  // only be called by GlobalContactManager
    void  do_build() final override;
    void  report_extent(GlobalContactManager::ContactExtentInfo& info);
    void  assemble(GlobalContactManager::ContactInfo& info);
    void  compute_energy(GlobalContactManager::EnergyInfo& info);
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
