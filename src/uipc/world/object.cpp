#include <uipc/world/object.h>

namespace uipc::world
{
std::string_view IObject::name() const noexcept
{
    return get_name();
}

Object::Object(std::string_view name) noexcept
    : m_name{name}
{
    m_geometries.reserve(4);
    m_rest_geometries.reserve(4);
}

std::string_view Object::get_name() const noexcept
{
    return m_name;
}

Object::Geometries::Geometries(Object& object) noexcept
    : m_object{object}
{
}

Object::CGeometries::CGeometries(const Object& object) noexcept
    : m_object{object}
{
}

Object::Geometries Object::geometries()
{
    return Geometries{*this};
}

Object::CGeometries Object::geometries() const
{
    return CGeometries{*this};
}
}  // namespace uipc::world
