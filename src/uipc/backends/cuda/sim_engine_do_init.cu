#include <sim_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <sim_system_auto_register.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void SimEngine::do_init(backend::WorldVisitor v)
{
    LogGuard guard;

    spdlog::info("do_init() called.");

    m_world_visitor = std::make_unique<backend::WorldVisitor>(v);

    // 1) Register all systems
    auto& funcs = SimSystemAutoRegister::internal_data().m_entries;
    for(auto& f : funcs)
    {
        auto uptr = f(*this);
        if(uptr)
            m_system_collection.create(std::move(uptr));
    }

    spdlog::info("Registered Systems:\n{}", m_system_collection);

    // 2) Build the relationships between systems
    m_state = SimEngineState::BuildSystems;
    for(auto&& [k, s] : m_system_collection.m_sim_systems)
        s->build();

    // 3) trigger the init_scene event, systems register their actions will be called here
    m_state = SimEngineState::InitScene;
    event_init_scene();
}
}  // namespace uipc::backend::cuda