#include <sim_engine.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    LogGuard guard;

    spdlog::info("do_advance() called.");
}
}  // namespace uipc::backend::cuda
