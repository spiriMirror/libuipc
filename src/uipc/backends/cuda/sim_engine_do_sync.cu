#include <sim_engine.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void SimEngine::do_sync()
{
    LogGuard guard;

    spdlog::info("do_sync() called.");
}
}  // namespace uipc::backend::cuda
