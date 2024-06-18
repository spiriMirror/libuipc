#include <sim_system.h>
#include <typeinfo>
#include <sim_engine.h>
#include <magic_enum.hpp>

namespace uipc::backend::cuda
{
SimSystem::SimSystem(SimEngine& sim_engine) noexcept
    : m_sim_engine(sim_engine)
{
}

void SimSystem::on_init_scene(std::function<void()>&& action)
{
    UIPC_ASSERT(m_sim_engine.m_state == SimEngineState::BuildSystems,
                "`on_init_scene()` can only be called in `build()`({}), but current state ({}).",
                magic_enum::enum_name(m_sim_engine.m_state),
                magic_enum::enum_name(m_sim_engine.m_state));
    m_sim_engine.m_on_init_scene.emplace_back(*this, std::move(action));
}

SimSystemCollection& SimSystem::collection()
{
    return m_sim_engine.m_system_collection;
}

WorldVisitor& SimSystem::world()
{
    return m_sim_engine.world();
}
}  // namespace uipc::backend::cuda
