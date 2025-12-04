#include <affine_body/global_external_force_manager.h>
#include <affine_body/affine_body_external_force_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalExternalForceManager);

void GlobalExternalForceManager::do_build()
{
    // Nothing to do in build phase
}

void GlobalExternalForceManager::register_reporter(AffineBodyExternalForceReporter* reporter)
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

void GlobalExternalForceManager::step()
{
    // Update all external force reporters
    // This happens AFTER animator step, so constraints have already updated body_id_to_external_wrench
    for(auto reporter : m_reporters.view())
    {
        reporter->step();
    }
}
}  // namespace uipc::backend::cuda
