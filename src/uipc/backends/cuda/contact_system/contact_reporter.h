#pragma once
#include <sim_system.h>
#include <contact_system/global_contact_manager.h>
namespace uipc::backend::cuda
{
class ContactReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_report_extent(GlobalContactManager::ContactExtentInfo& info) = 0;
    virtual void do_assemble(GlobalContactManager::ContactInfo& info) = 0;

  private:
    friend class GlobalContactManager;
    void  report_extent(GlobalContactManager::ContactExtentInfo& info);
    void  assemble(GlobalContactManager::ContactInfo& info);
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
