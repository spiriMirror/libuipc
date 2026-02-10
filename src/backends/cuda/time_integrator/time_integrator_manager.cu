#include <time_integrator/time_integrator_manager.h>
#include <time_integrator/time_integrator.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(TimeIntegratorManager);

void TimeIntegratorManager::do_build()
{
    auto dt_attr = world().scene().config().find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];
}

void TimeIntegratorManager::init()
{
    // Initialize the time integrators
    for(auto& integrator : m_impl.time_integrators.view())
    {
        integrator->init();
    }
}
void TimeIntegratorManager::predict_dof()
{
    // Predict the degrees of freedom for each time integrator
    for(auto& integrator : m_impl.time_integrators.view())
    {
        PredictDofInfo info;
        info.m_dt = m_impl.dt;
        integrator->predict_dof(info);
    }
}

void TimeIntegratorManager::update_state()
{
    // Update the state for each time integrator
    for(auto& integrator : m_impl.time_integrators.view())
    {
        UpdateVelocityInfo info;
        info.m_dt = m_impl.dt;
        integrator->update_state(info);
    }
}

void TimeIntegratorManager::add_integrator(TimeIntegrator* integrator)
{
    UIPC_ASSERT(integrator != nullptr, "Integrator cannot be null.");
    check_state(SimEngineState::BuildSystems, "add_integrator()");
    m_impl.time_integrators.register_sim_system(*integrator);
}
}  // namespace uipc::backend::cuda