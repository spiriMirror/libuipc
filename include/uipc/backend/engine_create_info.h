#pragma once
#include <string_view>

namespace uipc::backend
{
class EngineCreateInfo
{
  public:
    std::string_view workspace;
};
}  // namespace uipc::backend

using EngineCreateInfo = uipc::backend::EngineCreateInfo;
