#include <sim_engine.h>
#include <log_pattern_guard.h>
#include <muda/launch/launch.h>

namespace uipc::backend::cuda
{
void SimEngine::do_sync()
{
    LogGuard guard;
    // Sync the device
    muda::wait_device();
}
}  // namespace uipc::backend::cuda
