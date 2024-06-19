#include <sim_engine.h>
#include <uipc/backends/module.h>

namespace uipc::backend::cuda
{
WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
}

SimEngineState SimEngine::state() const noexcept
{
    return m_state;
}

void SimEngine::event_init_scene()
{
    for(auto& action : m_on_init_scene)
        action();
}

void SimEngine::event_rebuild_scene()
{
    for(auto& action : m_on_rebuild_scene)
        action();
}

void SimEngine::event_write_scene()
{
    for(auto& action : m_on_write_scene)
        action();
}

void SimEngine::register_all_systems() 
{
    auto& funcs = SimSystemAutoRegister::internal_data().m_entries;
    for(auto& f : funcs)
    {
        auto uptr = f(*this);
        if(uptr)
            m_system_collection.create(std::move(uptr));
    }

    spdlog::info("Registered Systems:\n{}", m_system_collection);
}
}  // namespace uipc::backend::cuda


UIPC_BACKEND_API UIPCEngineInterface* uipc_create_engine()
{
    return new uipc::backend::cuda::SimEngine();
}

UIPC_BACKEND_API void uipc_destroy_engine(UIPCEngineInterface* engine)
{
    delete engine;
}