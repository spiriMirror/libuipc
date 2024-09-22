#include <sim_engine.h>
#include <log_pattern_guard.h>
#include <muda/launch/launch.h>

namespace uipc::backend::cuda
{
void SimEngine::do_sync()
{
    LogGuard guard;
    try
    {
        // Sync the device
        muda::wait_device();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine sync error: {}", e.what());
    }
}
}  // namespace uipc::backend::cuda
