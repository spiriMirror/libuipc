#include <backends/common/sim_system_auto_register.h>


namespace uipc::backend
{
SimSystemAutoRegister::SimSystemAutoRegister(Creator&& reg)
{
    creators().entries.push_back(std::move(reg));
}

auto SimSystemAutoRegister::creators() -> Creators&
{
    static Creators data;
    return data;
}
}  // namespace uipc::backend
