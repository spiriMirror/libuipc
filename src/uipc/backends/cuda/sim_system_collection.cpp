#include <sim_system_collection.h>
#include <typeinfo>
#include <uipc/common/log.h>


namespace uipc::backend::cuda
{
void SimSystemCollection::create(U<ISimSystem> system)
{
    auto& s   = *system;
    U64   tid = typeid(s).hash_code();
    auto  it  = m_sim_systems.find(tid);
    UIPC_ASSERT(it == m_sim_systems.end(),
                "SimSystem ({}) already exists, yours {}, why can it happen?",
                it->second->name(),
                s.name());

    m_sim_systems.insert({tid, std::move(system)});
}

Json SimSystemCollection::to_json() const
{
    Json j = Json::array();
    for(const auto& [key, value] : m_sim_systems)
        j.push_back(value->to_json());
    return j;
}
}  // namespace uipc::backend::cuda

namespace fmt
{
appender formatter<uipc::backend::cuda::SimSystemCollection>::format(
    const uipc::backend::cuda::SimSystemCollection& s, format_context& ctx) const
{
    int i = 0;
    int n = s.m_sim_systems.size();
    for(const auto& [key, value] : s.m_sim_systems)
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