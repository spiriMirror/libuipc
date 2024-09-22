#pragma once
#include <uipc/common/macro.h>
#include <memory_resource>


class UIPC_CORE_API UIPCModuleInitInfo
{
  public:
    std::string_view           module_name;
    std::pmr::memory_resource* memory_resource = nullptr;
};