#pragma once
#include <uipc/common/macro.h>
#include <memory_resource>


class UIPC_CORE_API UIPCModuleInitInfo
{
  public:
    std::pmr::memory_resource* memory_resource = nullptr;
};