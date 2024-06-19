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

    m_world_visitor = std::make_unique<backend::WorldVisitor>(v);

    // 1) Register all systems
    m_state = SimEngineState::RegisterSystems;
    register_all_systems();

    // 2) Build the relationships between systems
    m_state = SimEngineState::BuildSystems;
    for(auto&& [k, s] : m_system_collection.m_sim_systems)
        s->build();

    // 3) Trigger the init_scene event, systems register their actions will be called here
    m_state = SimEngineState::InitScene;
    event_init_scene();

    // 4) Any creation and deletion of objects after this point will be pending
    auto scene_visitor = m_world_visitor->scene();
    scene_visitor.begin_pending();
}
}  // namespace uipc::backend::cuda