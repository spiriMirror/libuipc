#pragma once
#include <uipc/common/macro.h>
namespace uipc::backend
{
class UIPC_CORE_API IReporter
{
  public:
    virtual ~IReporter() = default;
};
}  // namespace uipc::backend
