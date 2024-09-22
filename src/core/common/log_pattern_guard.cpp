#include <uipc/common/log_pattern_guard.h>

namespace uipc
{
LogPatternGuard::LogPatternGuard(std::string_view pattern) noexcept
{
    spdlog::set_pattern(fmt::format("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [{}] %v", pattern));
}

LogPatternGuard::~LogPatternGuard() noexcept
{
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
}
}  // namespace uipc
