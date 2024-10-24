#include <diff_sim/global_diff_sim_manager.h>
#include <diff_sim/diff_sim_subsystem.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalDiffSimManager);

void GlobalDiffSimManager::Impl::init()
{
    auto subsystem_view = diff_sim_subsystems.view();
    for(auto&& [i, system] : enumerate(subsystem_view))
        system->m_index = i;
}


void GlobalDiffSimManager::init() {}

void GlobalDiffSimManager::do_build()
{
    if(!world().scene().info()["diff_sim"]["enable"] == true)
    {
        throw SimSystemException("differentiable simulation is not needed");
    }

    on_write_scene([&] { write_scene(); });
}

void GlobalDiffSimManager::write_scene() {}

void GlobalDiffSimManager::add_subsystem(DiffSimSubsystem* subsystem)
{
    m_impl.diff_sim_subsystems.register_subsystem(*subsystem);
}
}  // namespace uipc::backend::cuda