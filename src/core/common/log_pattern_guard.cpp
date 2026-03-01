#include <uipc/common/log_pattern_guard.h>

namespace uipc
{
// spdlog uses a Meyers-singleton registry.  During process exit the
// registry may already be destroyed when C++ statics or Python
// destructor chains run.  Calling spdlog::set_pattern() at that
// point dereferences freed memory (access violation on Windows).
// We guard with spdlog::default_logger_raw() which returns nullptr
// once the registry is gone.

LogPatternGuard::LogPatternGuard(std::string_view pattern) noexcept
{
    if(spdlog::default_logger_raw())
        spdlog::set_pattern(fmt::format("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [{}] %v", pattern));
}

LogPatternGuard::~LogPatternGuard() noexcept
{
    if(spdlog::default_logger_raw())
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
}
}  // namespace uipc
