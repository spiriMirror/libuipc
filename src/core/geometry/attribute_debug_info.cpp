#include <uipc/geometry/attribute_debug_info.h>


namespace uipc::geometry
{
std::string& AttributeDebugInfo::thread_local_last_not_found_name()
{
    static thread_local std::string last_not_found_name;
    return last_not_found_name;
}

Json& AttributeDebugInfo::thread_local_extras()
{
    static thread_local Json extras_json;
    return extras_json;
}
}  // namespace uipc::geometry
