#include <sim_engine.h>
#include <uipc/backends/module.h>

namespace uipc::backend::cuda
{
auto SimEngine::device_common() noexcept -> DeviceCommon&
{
    return *m_device_common;
}

WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
}

void SimEngine::event_init_scene()
{
    for(auto& action : m_on_init_scene)
        action();
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
