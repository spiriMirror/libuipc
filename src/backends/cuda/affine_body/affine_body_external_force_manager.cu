#include <affine_body/affine_body_external_force_manager.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_external_force_reporter.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void affine_body_external_force_manager_clear_kernel(
        cuda_tool::BufferView<Vector12> forces, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        forces(i).setZero();
    }

    __global__ void affine_body_external_force_manager_step_kernel(
        cuda_tool::CBufferView<Vector12>    forces,
        cuda_tool::BufferView<Vector12>     force_accs,
        cuda_tool::CBufferView<Matrix12x12> masses_inv,
        int                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const auto& F     = forces(i);
        const auto& M_inv = masses_inv(i);

        // Compute acceleration: a = M^{-1} * F (like gravity)
        force_accs(i) = M_inv * F;
    }
}  // namespace

REGISTER_SIM_SYSTEM(AffineBodyExternalForceManager);

void AffineBodyExternalForceManager::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
}

void AffineBodyExternalForceManager::register_reporter(AffineBodyExternalForceReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "register_reporter");
    m_impl.m_reporters.register_sim_system(*reporter);
}

void AffineBodyExternalForceManager::Impl::clear()
{
    // Clear external force buffer BEFORE constraints write to it
    // Read Write BufferView
    auto external_forces =
        affine_body_dynamics->m_impl.body_id_to_external_force.view();

    int n = (int)external_forces.size();
    if(n > 0)
    {
        auto k = affine_body_external_force_manager_clear_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            external_forces, n);
    }
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
    auto& abd = affine_body_dynamics->m_impl;
    // Read Write BufferView
    auto force_accs = abd.body_id_to_external_force_acc.view();

    // Read Only BufferViews
    auto forces     = affine_body_dynamics->body_external_forces();
    auto masses_inv = affine_body_dynamics->body_mass_invs();

    SizeT body_count = forces.size();

    int n = (int)body_count;
    if(n > 0)
    {
        auto k = affine_body_external_force_manager_step_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            forces.cview(), force_accs, masses_inv.cview(), n);
    }
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

cuda_tool::BufferView<Vector12> AffineBodyExternalForceManager::ExternalForceInfo::external_forces() noexcept
{
    return m_impl->affine_body_dynamics->m_impl.body_id_to_external_force.view();
}
}  // namespace uipc::backend::cuda
