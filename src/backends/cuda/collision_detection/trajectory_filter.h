#pragma once
#include <sim_system.h>
#include <collision_detection/global_trajectory_filter.h>

namespace uipc::backend::cuda
{
class TrajectoryFilter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_detect(GlobalTrajectoryFilter::DetectInfo& info) = 0;
    virtual void do_filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info) = 0;
    virtual void do_filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info) = 0;
    virtual void do_record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo&) = 0;
    virtual void do_label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info) = 0;
    virtual void do_clear_friction_candidates() = 0;

  private:
    friend class GlobalTrajectoryFilter;

    void detect(GlobalTrajectoryFilter::DetectInfo& info);
    void filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info);
    void filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info);
    void record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info);
    void label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info);
    void clear_friction_candidates();
};
}  // namespace uipc::backend::cuda
