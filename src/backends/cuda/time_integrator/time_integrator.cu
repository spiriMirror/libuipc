#include <time_integrator/time_integrator.h>

namespace uipc::backend::cuda
{
void TimeIntegrator::do_build()
{
    auto& manager = require<TimeIntegratorManager>();

    BuildInfo info;
    do_build(info);

    manager.add_integrator(this);
}

void TimeIntegrator::init()
{
    InitInfo info;
    do_init(info);
}

void TimeIntegrator::predict_dof(PredictDofInfo& info)
{
    do_predict_dof(info);
}

void TimeIntegrator::update_state(UpdateStateInfo& info)
{
    do_update_state(info);
}
}  // namespace uipc::backend::cuda