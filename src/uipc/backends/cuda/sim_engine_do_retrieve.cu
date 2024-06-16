#include <sim_engine.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void SimEngine::do_retrieve()
{
    LogGuard guard;

    spdlog::info("do_retrieve() called.");
}
}  // namespace uipc::backend::cuda
