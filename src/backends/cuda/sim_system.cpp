#include <sim_system.h>
#include <typeinfo>
#include <sim_engine.h>
#include <magic_enum.hpp>

namespace uipc::backend::cuda
{
void SimSystem::check_state(SimEngineState state, std::string_view function_name) noexcept
{
    UIPC_ASSERT(engine().m_state == state,
                "`{}` can only be called in `{}`({}), but current state ({}).",
                function_name,
                magic_enum::enum_name(state),
                magic_enum::enum_name(state),
                magic_enum::enum_name(engine().m_state));
}

void SimSystem::on_init_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_init_scene()");
    engine().m_on_init_scene.register_action(*this, std::move(action));
}

void SimSystem::on_rebuild_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_rebuild_scene()");
    engine().m_on_rebuild_scene.register_action(*this, std::move(action));
}

void SimSystem::on_write_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_write_scene()");
    engine().m_on_write_scene.register_action(*this, std::move(action));
}

SimEngine& SimSystem::engine() noexcept
{
    return static_cast<SimEngine&>(Base::engine());
}

WorldVisitor& SimSystem::world() noexcept
{
    return engine().world();
}
}  // namespace uipc::backend::cuda
