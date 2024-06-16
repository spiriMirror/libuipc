#include <sim_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <sim_system_auto_register.h>
#include <log_pattern_guard.h>
#include <sim_system_auto_register.h>

namespace uipc::backend::cuda
{
void SimEngine::do_init(backend::WorldVisitor v)
{
    LogGuard guard;

    spdlog::info("do_init() called.");
    m_world_visitor = std::make_unique<backend::WorldVisitor>(v);
    auto& funcs     = SimSystemAutoRegister::internal_data().m_entries;
    for(auto& f : funcs)
    {
        auto uptr = f(*this);
        if(uptr)
            m_system_collection.create(std::move(uptr));
    }

    spdlog::info("Registered Systems:\n{}", m_system_collection);

    for(auto&& [k, s] : m_system_collection.m_sim_systems)
    {
        s->build();
    }
}

auto SimEngine::device_common() noexcept -> DeviceCommon&
{
    return *m_device_common;
}

WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
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
