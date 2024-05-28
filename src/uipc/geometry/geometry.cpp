#include <uipc/geometry/geometry.h>
#include <Eigen/Geometry>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
std::string_view IGeometry::type() const
{
    return get_type();
}

Geometry::MetaAttributes::MetaAttributes(AttributeCollection& attributes)
    : m_attributes(attributes)
{
}

Geometry::InstanceAttributes::InstanceAttributes(AttributeCollection& attributes)
    : m_attributes(attributes)
{
}

void Geometry::InstanceAttributes::resize(size_t size)
{
    m_attributes.resize(size);
}

void Geometry::InstanceAttributes::reserve(size_t size)
{
    m_attributes.reserve(size);
}

void Geometry::InstanceAttributes::clear()
{
    m_attributes.clear();
}

SizeT Geometry::InstanceAttributes::size() const
{
    return m_attributes.size();
}

void Geometry::InstanceAttributes::destroy(std::string_view name)
{
    m_attributes.destroy(name);
}

Geometry::Geometry()
{
    m_meta.resize(1);      // only one meta for one geometry
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

auto Geometry::instances() -> InstanceAttributes
{
    return InstanceAttributes{m_intances};
}
}  // namespace uipc::geometry