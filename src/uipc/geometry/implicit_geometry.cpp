#include <uipc/geometry/implicit_geometry.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
ImplicitGeometry::ImplicitGeometry()
{
    // Set the implicit geometry UID to 0 (Empty).
    m_meta.create<U64, false>(builtin::implicit_geometry_uid, 0ull);
}

std::string_view ImplicitGeometry::get_type() const noexcept
{
    return uipc::builtin::ImplicitGeometry;
}
}  // namespace uipc::geometry
