#pragma once
#include <sim_system.h>
#include <collision_detection/global_ccd_filter.h>
namespace uipc::backend::cuda
{
class CCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void filter_toi(GlobalCCDFilter::FilterInfo& info) = 0;

  private:
    friend class GlobalCCDFilter;
};
}  // namespace uipc::backend::cuda
