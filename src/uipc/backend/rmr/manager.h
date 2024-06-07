#pragma once
#include <uipc/common/macro.h>
namespace uipc::backend
{
class UIPC_CORE_API IManager
{
  public:
    virtual ~IManager() = default;
};
}  // namespace uipc::backend
