#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>
#include <uipc/common/json.h>

namespace uipc::geometry
{
class UIPC_CORE_API AttributeDebugInfo
{
  public:
    static std::string& thread_local_last_not_found_name();
    static Json&        thread_local_extras();
};
}  // namespace uipc::geometry