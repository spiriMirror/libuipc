#include <collision_detection/trajectory_filter.h>

namespace uipc::backend::cuda
{
void TrajectoryFilter::detect(GlobalTrajectoryFilter::DetectInfo& info)
{
    do_detect(info);
}

void TrajectoryFilter::filter_active(GlobalTrajectoryFilter::FilterActiveInfo& info)
{
    do_filter_active(info);
}

void TrajectoryFilter::filter_toi(GlobalTrajectoryFilter::FilterTOIInfo& info)
{
    do_filter_toi(info);
}

void TrajectoryFilter::record_friction_candidates(GlobalTrajectoryFilter::RecordFrictionCandidatesInfo& info)
{
    do_record_friction_candidates(info);
}

void TrajectoryFilter::label_active_vertices(GlobalTrajectoryFilter::LabelActiveVerticesInfo& info)
{
    do_label_active_vertices(info);
}

void TrajectoryFilter::clear_friction_candidates()
{
    do_clear_friction_candidates();
}
}  // namespace uipc::backend::cuda
