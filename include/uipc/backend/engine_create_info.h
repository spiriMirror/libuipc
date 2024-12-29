#pragma once
#include <string_view>
#include <uipc/common/dllexport.h>
#include <uipc/common/json.h>
namespace uipc::backend
{
class UIPC_CORE_API EngineCreateInfo
{
  public:
    std::string_view workspace;
    Json             config;
};
}  // namespace uipc::backend

using EngineCreateInfo = uipc::backend::EngineCreateInfo;
