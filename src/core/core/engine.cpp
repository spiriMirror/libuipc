#include <uipc/core/engine.h>
#include <uipc/core/internal/engine.h>

namespace uipc::core
{
Engine::Engine(std::string_view backend_name, std::string_view workspace, const Json& config)
    : m_internal{uipc::make_unique<internal::Engine>(backend_name, workspace, config)}
{
}

Engine::~Engine() {}

std::string_view Engine::backend_name() const noexcept
{
    return m_internal->backend_name();
}

std::string_view Engine::workspace() const noexcept
{
    return m_internal->workspace();
}

EngineStatusCollection& Engine::status()
{
    return m_internal->status();
}

const FeatureCollection& Engine::features()
{
    return m_internal->features();
}

Json Engine::to_json() const
{
    return m_internal->to_json();
}

Json Engine::default_config()
{
    return internal::Engine::default_config();
}
}  // namespace uipc::core