#pragma once
#include <spdlog/spdlog.h>
#include <uipc/common/macro.h>
namespace uipc
{
class UIPC_CORE_API LogPatternGuard
{
  public:
    LogPatternGuard(std::string_view pattern) noexcept;
    ~LogPatternGuard() noexcept;
};
}  // namespace uipc
