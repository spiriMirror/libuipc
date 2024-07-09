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

void SimSystem::check_state(SimEngineState state, std::string_view function_name) noexcept
{
    UIPC_ASSERT(m_sim_engine.m_state == state,
                "`{}` can only be called in `{}`({}), but current state ({}).",
                function_name,
                magic_enum::enum_name(state),
                magic_enum::enum_name(state),
                magic_enum::enum_name(m_sim_engine.m_state));
}

Json SimSystem::do_to_json() const
{
    Json j;
    j["name"] = name();
    j["deps"] = Json::array();
    for(const auto& dep : m_dependencies)
    {
        j["deps"].push_back(dep->name());
    }
    j["engine_aware"] = m_engine_aware;
    j["valid"]        = m_valid;
    return j;
}

void SimSystem::on_init_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_init_scene()");
    m_sim_engine.m_on_init_scene.emplace_back(*this, std::move(action));
}

SimSystemCollection& SimSystem::collection() noexcept
{
    return m_sim_engine.m_system_collection;
}

void SimSystem::set_engine_aware() noexcept
{
    m_engine_aware = true;
}

bool SimSystem::get_engine_aware() const noexcept
{
    return m_engine_aware;
}

void SimSystem::set_invalid() noexcept
{
    m_valid = false;
}

SimSystem* SimSystem::find_system(SimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            ptr = nullptr;
        }
        else
        {
            m_dependencies.push_back(ptr);
        }
    }
    return ptr;
}

SimSystem* SimSystem::require_system(SimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            set_invalid();
            throw SimSystemException(fmt::format("System {} is invalid", ptr->name()));
        }
        else
        {
            m_dependencies.push_back(ptr);
        }
    }
    return ptr;
}

bool SimSystem::get_valid() const noexcept
{
    return m_valid;
}

void SimSystem::on_rebuild_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_rebuild_scene()");
    m_sim_engine.m_on_rebuild_scene.emplace_back(*this, std::move(action));
}

void SimSystem::on_write_scene(std::function<void()>&& action) noexcept
{
    check_state(SimEngineState::BuildSystems, "on_write_scene()");
    m_sim_engine.m_on_write_scene.emplace_back(*this, std::move(action));
}

WorldVisitor& SimSystem::world() noexcept
{
    return m_sim_engine.world();
}
}  // namespace uipc::backend::cuda
