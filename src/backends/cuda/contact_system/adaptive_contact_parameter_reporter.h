#pragma once
#include <sim_system.h>
#include <contact_system/global_contact_manager.h>

namespace uipc::backend::cuda
{
class AdaptiveContactParameterReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class InitInfo
    {
      public:
    };

    class BuildInfo
    {
      public:
    };

    using AdaptiveParameterInfo = GlobalContactManager::AdaptiveParameterInfo;

  protected:
    virtual void do_init(InitInfo& info)                            = 0;
    virtual void do_build(BuildInfo& info)                          = 0;
    virtual void do_compute_parameters(AdaptiveParameterInfo& info) = 0;

  private:
    virtual void do_build() override;

    friend class GlobalContactManager;
    void init();
    void compute_parameters(AdaptiveParameterInfo& info);
};
}  // namespace uipc::backend::cuda