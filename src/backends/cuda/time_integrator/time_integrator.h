#pragma once
#include <sim_system.h>
#include <time_integrator/time_integrator_manager.h>

namespace uipc::backend::cuda
{
class TimeIntegrator : public SimSystem
{
  public:
    using PredictDofInfo  = TimeIntegratorManager::PredictDofInfo;
    using UpdateStateInfo = TimeIntegratorManager::UpdateStateInfo;

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
    virtual void do_init(InitInfo& info)                = 0;
    virtual void do_build(BuildInfo& info)              = 0;
    virtual void do_predict_dof(PredictDofInfo& info)   = 0;
    virtual void do_update_state(UpdateStateInfo& info) = 0;

  private:
    friend class TimeIntegratorManager;
    void do_build() override final;
    void init();
    void predict_dof(PredictDofInfo&);
    void update_state(UpdateStateInfo&);
};
}  // namespace uipc::backend::cuda