#pragma once
#include <uipc/common/log_pattern_guard.h>

namespace uipc::backend::cuda
{
class LogGuard : public uipc::LogPatternGuard
{
  public:
    LogGuard() noexcept;
    ~LogGuard() noexcept = default;
};
}  // namespace uipc::backend::cuda
