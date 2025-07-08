#include <uipc/geometry/utils/constitution_type.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_uid_collection.h>

namespace uipc::geometry
{
static constexpr std::string_view None = "None";

std::string constitution_type(const Geometry& geo)
{
    auto uid = geo.meta().find<U64>(builtin::constitution_uid);
    if(!uid)
        return std::string{None};
    U64 uid_v = uid->view()[0];
    if(!builtin::ConstitutionUIDCollection::instance().exists(uid_v))
        return std::string{None};
    auto& uid_info = builtin::ConstitutionUIDCollection::instance().find(uid_v);
    return uid_info.type;
}
}  // namespace uipc::geometry