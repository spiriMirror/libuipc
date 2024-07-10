#include <uipc/backends/sim_system_auto_register.h>


namespace uipc::backend
{
SimSystemAutoRegister::SimSystemAutoRegister(std::function<U<ISimSystem>(SimEngine&)>&& reg)
{
    internal_data().m_entries.push_back(std::move(reg));
}

SimSystemAutoRegisterInternalData& SimSystemAutoRegister::internal_data()
{
    static SimSystemAutoRegisterInternalData data;
    return data;
}
}  // namespace uipc::backend::cuda
