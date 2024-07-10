#include <uipc/backends/sim_system.h>
#include <uipc/backends/sim_engine.h>
#include <typeinfo>

namespace uipc::backend
{
SimSystem::SimSystem(SimEngine& sim_engine) noexcept
    : m_sim_engine(sim_engine)
{
}

void SimSystem::check_state(std::string_view function_name) noexcept
{
    UIPC_ASSERT(m_is_building,
                "`{}` can only be called when the SimEngine builds systems",
                function_name);
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

SimEngine& SimSystem::engine() noexcept
{
    return m_sim_engine;
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
            throw SimSystemException(fmt::format("SimSystem [{}] is invalid", ptr->name()));
        }
        else
        {
            m_dependencies.push_back(ptr);
        }
    }
    return ptr;
}

void SimSystem::set_building(bool b) noexcept
{
    m_is_building = b;
}

bool SimSystem::get_is_building() const noexcept
{
    return m_is_building;
}

bool SimSystem::get_valid() const noexcept
{
    return m_valid;
}
}  // namespace uipc::backend
