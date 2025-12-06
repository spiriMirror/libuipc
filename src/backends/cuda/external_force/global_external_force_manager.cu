#include <external_force/global_external_force_manager.h>
#include <external_force/external_force_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalExternalForceManager);

void GlobalExternalForceManager::do_build()
{
    // Nothing to do in build phase
}

void GlobalExternalForceManager::register_reporter(ExternalForceReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "register_reporter");
    m_reporters.register_subsystem(*reporter);
}

void GlobalExternalForceManager::init()
{
    // Initialize all reporters
    for(auto reporter : m_reporters.view())
    {
        reporter->init();
    }
}

void GlobalExternalForceManager::clear()
{
    // Clear all force buffers BEFORE constraints write to them
    for(auto reporter : m_reporters.view())
    {
        reporter->clear();
    }
}

void GlobalExternalForceManager::step()
{
    // Compute accelerations AFTER constraints have written forces
    for(auto reporter : m_reporters.view())
    {
        reporter->step();
    }
}
}  // namespace uipc::backend::cuda
