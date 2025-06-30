#include <backends/common/sim_system.h>
#include <backends/common/sim_engine.h>
#include <typeinfo>
#include <uipc/common/demangle.h>

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
    j["name"]        = name();
    j["strong_deps"] = Json::array();
    for(const auto& dep : m_strong_dependencies)
    {
        j["strong_deps"].push_back(dep->name());
    }
    j["weak_deps"] = Json::array();
    for(const auto& dep : m_weak_dependencies)
    {
        j["weak_deps"].push_back(dep->name());
    }
    j["engine_aware"] = m_engine_aware;
    j["valid"]        = m_valid;
    return j;
}

SimEngine& SimSystem::engine() noexcept
{
    return m_sim_engine;
}

std::string_view SimSystem::workspace() const noexcept
{
    return m_sim_engine.workspace();
}

core::FeatureCollection& SimSystem::features() noexcept
{
    return m_sim_engine.features();
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

span<ISimSystem* const> SimSystem::get_strong_dependencies() const noexcept
{
    return m_strong_dependencies;
}

span<ISimSystem* const> SimSystem::get_weak_dependencies() const noexcept
{
    return m_weak_dependencies;
}

SimSystem* SimSystem::find_system(SimSystem* ptr)
{
    if(ptr)
    {
        // always record deps
        m_weak_dependencies.push_back(ptr);

        if(!ptr->is_valid())
        {
            ptr = nullptr;
        }
    }
    return ptr;
}

SimSystem* SimSystem::require_system(SimSystem* ptr)
{
    if(ptr)
    {
        // always record deps
        m_strong_dependencies.push_back(ptr);

        if(!ptr->is_valid())
        {
            set_invalid();
            throw SimSystemException(fmt::format("SimSystem [{}] is invalid", ptr->name()));
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

std::string_view SimSystem::get_name() const noexcept
{
    if(m_name.empty())
        m_name = uipc::demangle(typeid(*this).name());
    return m_name;
}

bool SimSystem::get_valid() const noexcept
{
    return m_valid;
}
}  // namespace uipc::backend
