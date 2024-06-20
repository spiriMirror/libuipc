#include <sim_engine.h>
#include <log_pattern_guard.h>
namespace uipc::backend::cuda
{
void SimEngine::do_retrieve()
{
    LogGuard guard;
    event_write_scene();
}
}  // namespace uipc::backend::cuda
