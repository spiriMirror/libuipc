#include <uipc/core/object.h>
#include <uipc/core/scene.h>

namespace uipc::core
{
std::string_view IObject::name() const noexcept
{
    return get_name();
}

IndexT IObject::id() const noexcept
{
    return get_id();
}

Object::Object(Scene& scene, IndexT id, std::string_view name) noexcept
    : m_scene{scene}
    , m_id{id}
    , m_name{name}
{
    m_geometry_ids.reserve(4);
}

std::string_view Object::get_name() const noexcept
{
    return m_name;
}

IndexT Object::get_id() const noexcept
{
    return m_id;
}

geometry::GeometryCollection& Object::geometry_collection() noexcept
{
    return m_scene.geometry_collection();
}

geometry::GeometryCollection& Object::rest_geometry_collection() noexcept
{
    return m_scene.rest_geometry_collection();
}

bool Object::scene_started() const noexcept
{
    return m_scene.is_started();
}

bool Object::scene_pending() const noexcept
{
    return m_scene.is_pending();
}

span<const IndexT> Object::Geometries::ids() && noexcept
{
    return m_object.m_geometry_ids;
}

Object::Geometries::Geometries(Object& object) noexcept
    : m_object{object}
{
}

span<const IndexT> Object::CGeometries::ids() && noexcept
{
    return m_object.m_geometry_ids;
}

Object::CGeometries::CGeometries(const Object& object) noexcept
    : m_object{object}
{
}

Object::Geometries Object::geometries() noexcept
{
    return Geometries{*this};
}

Object::CGeometries Object::geometries() const noexcept
{
    return CGeometries{*this};
}

Object::~Object() {}
}  // namespace uipc::core
