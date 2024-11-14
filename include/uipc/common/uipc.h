#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/json.h>

namespace uipc
{
UIPC_CORE_API void        init(const Json& config);
UIPC_CORE_API Json        default_config();
UIPC_CORE_API const Json& config();
}  // namespace uipc
