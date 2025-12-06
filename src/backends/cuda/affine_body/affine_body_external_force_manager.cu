#include <affine_body/affine_body_external_force_manager.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_external_force_reporter.h>

namespace uipc::backend::cuda
{
namespace
{
    // Helper function to clear external forces (needs to be outside class for CUDA lambda)
    void clear_external_forces(AffineBodyDynamics* dynamics)
    {
        auto external_forces = dynamics->body_external_forces();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(external_forces.size(),
                   [forces = external_forces.viewer().name("forces")] __device__(int i) mutable
                   {
                       forces(i).setZero();
                   });
    }
}  // namespace

REGISTER_SIM_SYSTEM(AffineBodyExternalForceManager);

void AffineBodyExternalForceManager::do_build(BuildInfo& info)
{
    affine_body_dynamics = &require<AffineBodyDynamics>();
}

void AffineBodyExternalForceManager::register_subreporter(AffineBodyExternalForceReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "register_subreporter");
    m_subreporters.register_subsystem(*reporter);
}

void AffineBodyExternalForceManager::do_init()
{
    // Initialize all sub-reporters
    for(auto reporter : m_subreporters.view())
    {
        reporter->init();
    }
}

void AffineBodyExternalForceManager::do_clear()
{
    // Clear external force buffer BEFORE constraints write to it
    clear_external_forces(affine_body_dynamics);
}

void AffineBodyExternalForceManager::do_step()
{
    // At this point, constraints have already written to external_force buffer
    // Now let all sub-reporters compute M^{-1} * F
    for(auto reporter : m_subreporters.view())
    {
        reporter->step();
    }
}
}  // namespace uipc::backend::cuda
