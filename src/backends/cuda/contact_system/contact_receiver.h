#pragma once
#include <sim_system.h>
#include <contact_system/global_contact_manager.h>

namespace uipc::backend::cuda
{
class ContactReceiver : public SimSystem
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
    virtual void do_init(InitInfo&);
    virtual void do_report(GlobalContactManager::ClassifyInfo& info) = 0;
    virtual void do_receive(GlobalContactManager::ClassifiedContactInfo& info) = 0;
    virtual void do_build(BuildInfo& info) = 0;

  private:
    friend class GlobalContactManager;
    virtual void do_build() final override;
    void         init();  // only be called by GlobalContactManager
    void         report(GlobalContactManager::ClassifyInfo& info);
    void         receive(GlobalContactManager::ClassifiedContactInfo& info);
    SizeT        m_index = ~0ull;
};
}  // namespace uipc::backend::cuda
