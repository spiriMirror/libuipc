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
    m_geometry.reserve(4);
    m_rest_geometry.reserve(4);
}

std::string_view Object::get_name() const noexcept
{
    return m_name;
}
}  // namespace uipc::world
