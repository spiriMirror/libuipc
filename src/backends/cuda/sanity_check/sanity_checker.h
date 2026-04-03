#pragma once
#include <sim_system.h>
#include <sanity_check/sanity_check_manager.h>

namespace uipc::backend::cuda
{
class SanityChecker : public SimSystem
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

    using CheckInfo = SanityCheckManager::CheckInfo;

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo& info)   = 0;
    virtual void do_check(CheckInfo& info) = 0;

  private:
    virtual void do_build() override;

    friend class SanityCheckManager;
    void init();
    void check(CheckInfo& info);
};
}  // namespace uipc::backend::cuda
