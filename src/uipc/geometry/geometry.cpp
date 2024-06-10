#include <uipc/geometry/geometry.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
std::string_view IGeometry::type() const noexcept
{
    return get_type();
}

Geometry::Geometry()
{
    m_meta.resize(1);      // only one meta for one geometries
    m_intances.resize(1);  // default only one instance
    Matrix4x4 I = Eigen::Transform<Float, 3, Eigen::Affine>::Identity().matrix();
    auto trans = m_intances.create<Matrix4x4, false>(builtin::transform, I);
}

AttributeSlot<Matrix4x4>& Geometry::transforms()
{
    return *m_intances.template find<Matrix4x4>(builtin::transform);
}

const AttributeSlot<Matrix4x4>& Geometry::transforms() const
{
    return *m_intances.template find<Matrix4x4>(builtin::transform);
}

auto Geometry::meta() -> MetaAttributes
{
    return MetaAttributes{m_meta};
}

auto Geometry::meta() const -> CMetaAttributes
{
    return CMetaAttributes{m_meta};
}

auto Geometry::instances() -> InstanceAttributes
{
    return InstanceAttributes{m_intances};
}
auto Geometry::instances() const -> CInstanceAttributes
{
    return CInstanceAttributes{m_intances};
}
}  // namespace uipc::geometry

namespace fmt
{
appender formatter<uipc::geometry::Geometry>::format(const uipc::geometry::Geometry& geo,
                                                     format_context& ctx)
{
    return fmt::format_to(ctx.out(),
                          R"(type:<{}>;
meta:{};
instances({}):{};)",
                          geo.type(),
                          geo.meta(),
                          geo.instances().size(),
                          geo.instances());
}
}  // namespace fmt
