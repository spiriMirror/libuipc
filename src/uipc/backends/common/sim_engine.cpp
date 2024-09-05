#include <uipc/backends/common/sim_engine.h>
#include <uipc/backends/common/sim_system_auto_register.h>
#include <uipc/backends/common/module.h>
#include <filesystem>
#include <fstream>

namespace uipc::backend
{
Json SimEngine::do_to_json() const
{
    Json j;
    j["sim_systems"] = m_system_collection.to_json();
    return j;
}

void SimEngine::build_systems()
{
    auto& funcs = SimSystemAutoRegister::creators().entries;
    for(auto& f : funcs)
    {
        auto uptr = f(*this);
        if(uptr)
            m_system_collection.create(std::move(uptr));
    }

    m_system_collection.build_systems();
}

void SimEngine::dump_system_info() const
{
    namespace fs = std::filesystem;

    spdlog::info("Built systems:\n{}", m_system_collection);

    auto     workspace = ModuleInfo::instance().workspace();
    fs::path p         = fs::absolute(fs::path{workspace} / "systems.json");
    {
        std::ofstream ofs(p);
        ofs << to_json().dump(4);
    }
    spdlog::info("System info dumped to {}", p.string());
}

span<ISimSystem* const> SimEngine::systems() noexcept
{
    return m_system_collection.systems();
}

std::string SimEngine::dump_path() const noexcept
{
    namespace fs = std::filesystem;

    fs::path p = std::string{workspace()};
    p /= "sim_data";
    p /= "";
    fs::exists(p) || fs::create_directories(p);
    return p.string();
}

ISimSystem* SimEngine::find_system(ISimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            ptr = nullptr;
        }
        else
        {
            ptr->set_engine_aware();
        }
    }
    return ptr;
}

ISimSystem* SimEngine::require_system(ISimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            throw SimEngineException(fmt::format("SimSystem [{}] is invalid", ptr->name()));
        }
        else
        {
            ptr->set_engine_aware();
        }
    }
    return ptr;
}

std::string_view SimEngine::workspace() const noexcept
{
    return ModuleInfo::instance().workspace();
}

bool SimEngine::do_dump()
{
    bool all_success = true;

    for(auto system : systems())
    {
        ISimSystem::DumpInfo info{frame(), Json::object()};
        all_success &= system->do_dump(info);

        if(!all_success)
        {
            spdlog::error("Failed to dump system [{}]", system->name());
            break;
        }
    }

    return all_success;
}
bool SimEngine::do_recover()
{
    bool all_success = true;

    // First try recover
    for(auto system : systems())
    {
        ISimSystem::RecoverInfo info{frame(), Json::object()};
        all_success &= system->try_recover(info);

        if(!all_success)
        {
            spdlog::warn("Try recovering system [{}] fails, so skip recovery.",
                         system->name());
            break;
        }
    }

    if(all_success)  // If all success, apply recover
    {
        for(auto system : systems())
        {
            ISimSystem::RecoverInfo info{frame(), Json::object()};
            system->apply_recover(info);
        }
    }
    else  // If any fails, clear all recover
    {
        for(auto system : systems())
        {
            ISimSystem::RecoverInfo info{frame(), Json::object()};
            system->clear_recover(info);
        }
    }

    return all_success;
}
}  // namespace uipc::backend