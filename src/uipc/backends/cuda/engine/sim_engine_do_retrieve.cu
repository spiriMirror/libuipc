#include <sim_engine.h>
#include <log_pattern_guard.h>
namespace uipc::backend::cuda
{
void SimEngine::do_retrieve()
{
    LogGuard guard;
    try
    {
        event_write_scene();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine retrieve error: {}", e.what());
    }
}
}  // namespace uipc::backend::cuda
