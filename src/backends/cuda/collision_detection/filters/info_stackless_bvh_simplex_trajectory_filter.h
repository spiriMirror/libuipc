#pragma once
#include <sim_system.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/filters/stackless_bvh_simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
class InfoStacklessBVHSimplexTrajectoryFilter final : public SimplexTrajectoryFilter
{
  public:
    using SimplexTrajectoryFilter::SimplexTrajectoryFilter;
    using Impl = StacklessBVHSimplexTrajectoryFilter::Impl;

    virtual muda::CBufferView<Vector2i> candidate_PTs() const noexcept override;
    virtual muda::CBufferView<Vector2i> candidate_EEs() const noexcept override;
    virtual muda::CBufferView<Float>    toi_PTs() const noexcept override;
    virtual muda::CBufferView<Float>    toi_EEs() const noexcept override;

  private:
    Impl m_impl;

    virtual void do_build(BuildInfo& info) override final;
    virtual void do_detect(DetectInfo& info) override final;
    virtual void do_filter_active(FilterActiveInfo& info) override final;
    virtual void do_filter_toi(FilterTOIInfo& info) override final;
};
}  // namespace uipc::backend::cuda
