#pragma once
#include <uipc/common/type_define.h>

namespace uipc::world
{
class Scene;

enum class SanityCheckResult : int
{
    Success = 0,
    Warning = 1,
    Error   = 2
};

class ISanityChecker
{
  public:
    U64               id() const noexcept;
    SanityCheckResult check() noexcept;

  protected:
    virtual U64               get_id() const noexcept   = 0;
    virtual SanityCheckResult do_check() noexcept = 0;
};
}  // namespace uipc::world
