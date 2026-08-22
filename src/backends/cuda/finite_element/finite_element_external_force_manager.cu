#include <finite_element/finite_element_external_force_manager.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_external_force_reporter.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FEMExternalForceManager_clear_kernel(cuda_tool::BufferView<Vector3> forces,
                                                         int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        forces(i).setZero();
    }

    __global__ void FEMExternalForceManager_step_kernel(cuda_tool::CBufferView<Vector3> forces,
                                                        cuda_tool::BufferView<Vector3> force_accs,
                                                        cuda_tool::CBufferView<Float> masses,
                                                        int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const Vector3& F = forces(i);
        Float          m = masses(i);

        // a = F / m (avoid division by zero for massless vertices)
        if(m > 0.0)
            force_accs(i) = F / m;
        else
            force_accs(i).setZero();
    }
}  // namespace

REGISTER_SIM_SYSTEM(FEMExternalForceManager);

void FEMExternalForceManager::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
}

void FEMExternalForceManager::register_reporter(FiniteElementExternalForceReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "register_reporter");
    m_impl.m_reporters.register_sim_system(*reporter);
}

void FEMExternalForceManager::Impl::clear()
{
    auto external_forces = finite_element_method->m_impl.vertex_external_forces.view();

    auto k = FEMExternalForceManager_clear_kernel;
    int  n = (int)external_forces.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            external_forces, n);
    }
}

void FEMExternalForceManager::Impl::step()
{
    ExternalForceInfo info{this};
    for(auto reporter : m_reporters.view())
    {
        reporter->step(info);
    }

    auto& fem = finite_element_method->m_impl;

    auto force_accs = fem.vertex_external_force_accs.view();
    auto forces     = fem.vertex_external_forces.view();
    auto masses     = finite_element_method->masses();

    auto k = FEMExternalForceManager_step_kernel;
    int  n = (int)forces.size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            forces.cview(), force_accs, masses.cview(), n);
    }
}

void FEMExternalForceManager::do_init()
{
    for(auto reporter : m_impl.m_reporters.view())
    {
        reporter->init();
    }
}

void FEMExternalForceManager::do_clear()
{
    m_impl.clear();
}

void FEMExternalForceManager::do_step()
{
    m_impl.step();
}

cuda_tool::BufferView<Vector3> FEMExternalForceManager::ExternalForceInfo::external_forces() noexcept
{
    return m_impl->finite_element_method->m_impl.vertex_external_forces.view();
}
}  // namespace uipc::backend::cuda
