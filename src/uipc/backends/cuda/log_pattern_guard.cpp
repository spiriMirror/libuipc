#include <log_pattern_guard.h>


namespace uipc::backend::cuda
{
LogGuard::LogGuard() noexcept
    : LogPatternGuard{"CudaEngine"}
{
}
}  // namespace uipc::backend::cuda
