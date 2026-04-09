#include <finite_element/finite_element_external_force_manager.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_external_force_reporter.h>

namespace uipc::backend::cuda
{
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
    auto external_forces =
        finite_element_method->m_impl.vertex_external_forces.view();

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(external_forces.size(),
               [forces = external_forces.viewer().name("forces")] __device__(int i) mutable
               { forces(i).setZero(); });
}

void FEMExternalForceManager::Impl::step()
{
    ExternalForceInfo info{this};
    for(auto reporter : m_reporters.view())
    {
        reporter->step(info);
    }

    using namespace muda;

    auto& fem = finite_element_method->m_impl;

    auto force_accs = fem.vertex_external_force_accs.view();
    auto forces     = fem.vertex_external_forces.view();
    auto masses     = finite_element_method->masses();

    SizeT vertex_count = forces.size();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(vertex_count,
               [forces     = forces.cviewer().name("forces"),
                force_accs = force_accs.viewer().name("force_accs"),
                masses     = masses.cviewer().name("masses")] __device__(int i)
               {
                   const Vector3& F = forces(i);
                   Float          m = masses(i);

                   // a = F / m (avoid division by zero for massless vertices)
                   if(m > 0.0)
                       force_accs(i) = F / m;
                   else
                       force_accs(i).setZero();
               });
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

muda::BufferView<Vector3> FEMExternalForceManager::ExternalForceInfo::external_forces() noexcept
{
    return m_impl->finite_element_method->m_impl.vertex_external_forces.view();
}
}  // namespace uipc::backend::cuda
