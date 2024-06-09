#include <sim_system_collection.h>
#include <uipc/common/log.h>


namespace uipc::backend::cuda
{
void SimSystemCollection::create(U<SimSystem> system)
{
    U64  tid = typeid(*system).hash_code();
    auto it  = m_sim_systems.find(tid);
    UIPC_ASSERT(it == m_sim_systems.end(), "SimSystem already exists, why can it happen?");

    m_sim_systems.insert({tid, std::move(system)});
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
        fmt::format_to(ctx.out(), "* {}{}", value->name(), ++i != n ? "\n" : "");
    }
    return ctx.out();
}
}  // namespace fmt