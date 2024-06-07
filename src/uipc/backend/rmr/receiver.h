#pragma once
#include <uipc/common/macro.h>
namespace uipc::backend
{
class UIPC_CORE_API IReceiver
{
  public:
    virtual ~IReceiver()   = default;
};
}  // namespace uipc::backend
