#include <active_set_system/al_stiffness_estimator.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda
{

void ALStiffnessEstimator::do_build()
{
    auto& active_set = require<GlobalActiveSetManager>();

    BuildInfo info;
    do_build(info);

    active_set.add_stiffness_estimator(this);
}

void ALStiffnessEstimator::estimate_mu(EstimateInfo& info)
{
    do_estimate_mu(info);
}

}  // namespace uipc::backend::cuda
