#pragma once
#include <sim_system.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda
{

class ALStiffnessEstimator : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    class BuildInfo
    {
    };
    using EstimateInfo = GlobalActiveSetManager::StiffnessEstimateInfo;

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_estimate_mu(EstimateInfo& info) = 0;

  private:
    virtual void do_build() override final;

    friend class GlobalActiveSetManager;
    void         estimate_mu(EstimateInfo& info);
};

}  // namespace uipc::backend::cuda
