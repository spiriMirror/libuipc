#include <backends/common/sim_system_collection.h>
#include <typeinfo>
#include <uipc/common/log.h>
#include <uipc/common/set.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/stack.h>
namespace uipc::backend
{
void SimSystemCollection::create(U<ISimSystem> system)
{
    UIPC_ASSERT(!built, "SimSystemCollection is already built, cannot create new system any more!");
    auto&    s   = *system;
    uint64_t tid = typeid(s).hash_code();
    auto     it  = m_sim_system_map.find(tid);
    UIPC_ASSERT(it == m_sim_system_map.end(),
                "SimSystem ({}) already exists, yours {}, why can it happen?",
                it->second->name(),
                s.name());

    m_sim_system_map.insert({tid, std::move(system)});
}

Json SimSystemCollection::to_json() const
{
    Json j = Json::array();
    for(const auto& [key, value] : m_sim_system_map)
        j.push_back(value->to_json());
    return j;
}

span<ISimSystem* const> SimSystemCollection::systems() const
{
    UIPC_ASSERT(built, "SimSystemCollection is not built yet! Call build_systems() first!");
    return m_valid_systems;
}

void SimSystemCollection::cleanup_invalid_systems()
{
    // remove invalid systems
    bool changed = false;

    auto check_valid = [](const ISimSystem* ss) -> bool
    {
        auto this_valid = ss->is_valid();
        // check in strong dependencies
        auto deps       = ss->strong_dependencies();
        bool deps_valid = true;
        for(auto dep : deps)
        {
            if(!dep->is_valid())
            {
                deps_valid = false;
                spdlog::debug("[{}] will be removed, because its dep [{}] is invalid",
                              ss->name(),
                              dep->name());
                break;
            }
        }
        return this_valid && deps_valid;
    };

    // clean all invalid sim system
    do
    {
        changed = false;
        for(auto it = m_sim_system_map.begin(); it != m_sim_system_map.end();)
        {
            if(!check_valid(it->second.get()))
            {
                it->second->set_invalid();
                m_invalid_systems.push_back(std::move(it->second));
                it      = m_sim_system_map.erase(it);
                changed = true;
            }
            else
                ++it;
        }
    } while(changed);
}

void SimSystemCollection::build_systems()
{
    for(auto&& [k, s] : m_sim_system_map)
        s->set_building(true);

    for(auto&& [k, s] : m_sim_system_map)
    {
        try
        {
            s->build();
        }
        catch(SimSystemException& e)
        {
            s->set_invalid();
            spdlog::debug("[{}] shutdown, reason: {}", s->name(), e.what());
        }
    }

    cleanup_invalid_systems();

    m_valid_systems.resize(m_sim_system_map.size());
    std::ranges::transform(m_sim_system_map,
                           m_valid_systems.begin(),
                           [](auto& p) { return p.second.get(); });

    for(auto&& s : m_valid_systems)
        s->set_building(false);
    for(auto&& s : m_invalid_systems)
        s->set_building(false);

    built = true;
}
}  // namespace uipc::backend

namespace fmt
{
appender formatter<uipc::backend::SimSystemCollection>::format(
    const uipc::backend::SimSystemCollection& s, format_context& ctx) const
{
    int i = 0;
    int n = s.m_sim_system_map.size();
    for(const auto& [key, value] : s.m_sim_system_map)
    {
        fmt::format_to(ctx.out(),
                       "{} {}{}",
                       value->is_engine_aware() ? ">" : "*",
                       value->name(),
                       ++i != n ? "\n" : "");
    }
    return ctx.out();
}
}  // namespace fmt