#include <none_sim_system.h>

namespace uipc::backend::none
{
REGISTER_SIM_SYSTEM(NoneSimSystem);

void NoneSimSystem::do_build()
{
    spdlog::info("[NoneEngine] NoneSimSystem call do_build()");
}
}  // namespace uipc::backend::none
