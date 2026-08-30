#include <backends/common/sim_system_auto_register.h>


namespace uipc::backend
{
SimSystemAutoRegister::SimSystemAutoRegister(std::string type_name, Creator&& creator)
{
    creators().entries.push_back(Entry{std::move(type_name), std::move(creator)});
}

auto SimSystemAutoRegister::creators() -> Creators&
{
    static Creators data;
    return data;
}
}  // namespace uipc::backend
