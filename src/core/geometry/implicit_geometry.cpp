#include <uipc/geometry/implicit_geometry.h>
#include <uipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
ImplicitGeometry::ImplicitGeometry()
    : Geometry()
{
    auto meta = find("meta");  // in base class
    UIPC_ASSERT(meta, "Meta attribute collection not found, why can it happen?");

    // Set the implicit geometry UID to 0 (Empty).
    meta->create<U64>(builtin::implicit_geometry_uid,
                      0ull,  // default empty = 0
                      false  // don't allow destroy
    );
}

const builtin::UIDInfo& ImplicitGeometry::uid_info() const noexcept
{
    U64 uid = meta().find<U64>(builtin::implicit_geometry_uid)->view()[0];
    return builtin::ImplicitGeometryUIDCollection::instance().find(uid);
}

std::string_view ImplicitGeometry::name() const noexcept
{
    return uid_info().name;
}

std::string_view ImplicitGeometry::get_type() const noexcept
{
    return uipc::builtin::ImplicitGeometry;
}

S<IGeometry> ImplicitGeometry::do_clone() const
{
    return uipc::make_shared<ImplicitGeometry>(*this);
}
}  // namespace uipc::geometry

namespace fmt
{
appender fmt::formatter<uipc::geometry::ImplicitGeometry>::format(
    const uipc::geometry::ImplicitGeometry& geometry, format_context& ctx) const
{
    return fmt::format_to(ctx.out(), "{}", static_cast<const uipc::geometry::Geometry&>(geometry));
}
}  // namespace fmt
