#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class SimplexDCDFilter;
class HalfPlaneDCDFilter;

class GlobalDCDFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        SimSystemSlot<SimplexDCDFilter>   simplex_dcd_filter;
        SimSystemSlot<HalfPlaneDCDFilter> half_plane_dcd_filter;
        SimActionCollection<void()>       detect_actions;
        bool                              friction_enabled = false;
    };

    void add_filter(SimplexDCDFilter* filter);
    void add_filter(HalfPlaneDCDFilter* filter);
    void add_filter(SimSystem* system, std::function<void()>&& detect_action);

    SimplexDCDFilter*   simplex_filter() const;
    HalfPlaneDCDFilter* half_plane_filter() const;

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;

    void detect();
    void detect_trajectory_candidates(Float alpha);
    void filter_candidates();

    void record_friction_candidates();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda