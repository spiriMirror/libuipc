#include <finite_element/fem_time_integrator.h>

namespace uipc::backend::cuda
{
void FEMTimeIntegrator::do_build(TimeIntegrator::BuildInfo& info)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();

    BuildInfo this_info;
    do_build(this_info);
}

void FEMTimeIntegrator::do_init(TimeIntegrator::InitInfo& info)
{
    InitInfo this_info;
    do_init(this_info);
}

void FEMTimeIntegrator::do_predict_dof(TimeIntegrator::PredictDofInfo& info)
{
    PredictDofInfo this_info{&m_impl, &info};
    do_predict_dof(this_info);
}

void FEMTimeIntegrator::do_update_state(TimeIntegrator::UpdateStateInfo& info)
{
    UpdateStateInfo this_info{&m_impl, &info};
    do_update_state(this_info);
}
}  // namespace uipc::backend::cuda
