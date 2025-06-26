#include <affine_body/abd_time_integrator.h>

namespace uipc::backend::cuda
{
void ABDTimeIntegrator::do_build(TimeIntegrator::BuildInfo& info)
{
    m_impl.affine_body_dynamics = require<AffineBodyDynamics>();

    BuildInfo this_info;
    do_build(this_info);
}

void ABDTimeIntegrator::do_init(TimeIntegrator::InitInfo& info)
{
    InitInfo this_info;
    do_init(this_info);
}

void ABDTimeIntegrator::do_predict_dof(TimeIntegrator::PredictDofInfo& info)
{
    PredictDofInfo this_info{&m_impl, &info};
    do_predict_dof(this_info);
}

void ABDTimeIntegrator::do_update_state(TimeIntegrator::UpdateStateInfo& info)
{
    UpdateStateInfo this_info{&m_impl, &info};
    do_update_state(this_info);
}
}  // namespace uipc::backend::cuda
