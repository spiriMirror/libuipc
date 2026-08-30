#include <backends/common/sim_system_collection.h>
#include <typeinfo>
#include <uipc/common/log.h>
#include <uipc/common/set.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/stack.h>
#include <backends/common/details/dependency_cycle.h>
namespace uipc::backend
{
void SimSystemCollection::create(U<ISimSystem> system)
{
    UIPC_ASSERT(!built, "SimSystemCollection is already built, cannot create new system any more!");
    auto&                 s = *system;
    const std::type_index tid{typeid(s)};
    auto                  it = m_sim_system_map.find(tid);
    UIPC_ASSERT(it == m_sim_system_map.end(),
                "SimSystem ({}) already exists, yours {}, why can it happen?",
                it->second->name(),
                s.name());

    m_registration_order.push_back(&s);
    m_sim_system_map.insert({tid, std::move(system)});
}

Json SimSystemCollection::to_json() const
{
    Json        j       = Json::array();
    const auto& systems = built ? m_valid_systems : m_registration_order;
    for(const auto* system : systems)
        j.push_back(system->to_json());
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
                logger::debug("[{}] will be removed, because its dep [{}] is invalid",
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
        for(auto* system : m_registration_order)
        {
            const std::type_index tid{typeid(*system)};
            auto                  it = m_sim_system_map.find(tid);
            if(it == m_sim_system_map.end())
                continue;

            if(!check_valid(system))
            {
                system->set_invalid();
                m_invalid_systems.push_back(std::move(it->second));
                m_sim_system_map.erase(it);
                changed = true;
            }
        }
    } while(changed);
}

void SimSystemCollection::build_systems()
{
    for(auto* system : m_registration_order)
        system->set_building(true);

    for(auto* system : m_registration_order)
    {
        try
        {
            system->build();
        }
        catch(SimSystemException& e)
        {
            system->set_invalid();
            logger::debug("[{}] shutdown, reason: {}", system->name(), e.what());
        }
    }

    cleanup_invalid_systems();

    m_valid_systems.reserve(m_sim_system_map.size());
    for(auto* system : m_registration_order)
    {
        if(system->is_valid())
            m_valid_systems.push_back(system);
    }

    for(auto* system : m_registration_order)
        system->set_building(false);

    const auto cycle = detail::find_dependency_cycle<ISimSystem*>(
        m_valid_systems,
        [](ISimSystem* system) { return system->strong_dependencies(); },
        [](ISimSystem* system) { return system->is_valid(); });
    if(!cycle.empty())
    {
        std::string cycle_path;
        for(SizeT i = 0; i < cycle.size(); ++i)
        {
            if(i != 0)
                cycle_path += " -> ";
            cycle_path += cycle[i]->name();
        }
        throw SimSystemException(
            fmt::format("Strong SimSystem dependency cycle detected: {}", cycle_path));
    }

    built = true;
}
}  // namespace uipc::backend

namespace fmt
{
appender formatter<uipc::backend::SimSystemCollection>::format(
    const uipc::backend::SimSystemCollection& s, format_context& ctx) const
{
    const auto& systems = s.built ? s.m_valid_systems : s.m_registration_order;
    int         i       = 0;
    int         n       = systems.size();
    for(const auto* system : systems)
    {
        fmt::format_to(ctx.out(),
                       "{} {}{}",
                       system->is_engine_aware() ? ">" : "*",
                       system->name(),
                       ++i != n ? "\n" : "");
    }
    return ctx.out();
}
}  // namespace fmt
