#include <affine_body/affine_body_external_force_manager.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_external_force_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyExternalForceManager);

void AffineBodyExternalForceManager::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
}

void AffineBodyExternalForceManager::register_reporter(AffineBodyExternalForceReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "register_reporter");
    m_impl.m_reporters.register_subsystem(*reporter);
}

void AffineBodyExternalForceManager::Impl::clear()
{
    // Clear external force buffer BEFORE constraints write to it
    // Read Write BufferView
    auto external_forces =
        affine_body_dynamics->m_impl.body_id_to_external_force.view();

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(external_forces.size(),
               [forces = external_forces.viewer().name("forces")] __device__(int i) mutable
               { forces(i).setZero(); });
}

void AffineBodyExternalForceManager::Impl::step()
{
    // Step all sub-reporters
    ExternalForceInfo info{this};
    for(auto reporter : m_reporters.view())
    {
        reporter->step(info);
    }

    // At this point, constraints have already written to external_force buffer
    // Now compute accelerations from external forces
    using namespace muda;

    auto& abd = affine_body_dynamics->m_impl;
    // Read Write BufferView
    auto force_accs = abd.body_id_to_external_force_acc.view();

    // Read Only BufferViews
    auto forces     = affine_body_dynamics->body_external_forces();
    auto masses_inv = affine_body_dynamics->body_mass_invs();

    SizeT body_count = forces.size();

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(body_count,
               [forces     = forces.cviewer().name("forces"),
                force_accs = force_accs.viewer().name("force_accs"),
                masses_inv = masses_inv.cviewer().name("masses_inv")] __device__(int i)
               {
                   const auto& F     = forces(i);
                   const auto& M_inv = masses_inv(i);

                   // Compute acceleration: a = M^{-1} * F (like gravity)
                   force_accs(i) = M_inv * F;
               });
}

void AffineBodyExternalForceManager::do_init()
{
    // Initialize all sub-reporters
    for(auto reporter : m_impl.m_reporters.view())
    {
        reporter->init();
    }
}

void AffineBodyExternalForceManager::do_clear()
{
    m_impl.clear();
}

void AffineBodyExternalForceManager::do_step()
{
    m_impl.step();
}

muda::BufferView<Vector12> AffineBodyExternalForceManager::ExternalForceInfo::external_forces() noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_external_force.view();
}
}  // namespace uipc::backend::cuda
