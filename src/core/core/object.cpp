#include <uipc/core/object.h>
#include <uipc/core/scene.h>
#include <uipc/core/internal/scene.h>
#include <uipc/core/object_snapshot.h>

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

Object::Object(internal::Scene& scene, IndexT id, std::string_view name) noexcept
    : m_scene{&scene}
    , m_id{id}
    , m_name{name}
{
    m_geometry_ids.reserve(4);
}

Object::Object() noexcept
    : m_scene{nullptr}
    , m_id{-1}
{
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
    return m_scene->geometries();
}

geometry::GeometryCollection& Object::rest_geometry_collection() noexcept
{
    return m_scene->rest_geometries();
}

bool Object::scene_started() const noexcept
{
    return m_scene->is_started();
}

bool Object::scene_pending() const noexcept
{
    return m_scene->is_pending();
}

void Object::build_from(span<const IndexT> geo_ids) noexcept
{
    m_geometry_ids.clear();
    m_geometry_ids.reserve(geo_ids.size());
    std::ranges::copy(geo_ids, std::back_inserter(m_geometry_ids));
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

void to_json(Json& j, const Object& object) noexcept
{
    j["id"]         = object.id();
    j["name"]       = object.name();
    j["geometries"] = object.geometries().ids();
}

void Object::update_from(const ObjectSnapshot& snapshot) noexcept
{
    m_id           = snapshot.m_id;
    m_name         = snapshot.m_name;
    m_geometry_ids = snapshot.m_geometries;
}

void from_json(const Json& j, Object& object) noexcept
{
    object.m_id           = j["id"].get<IndexT>();
    object.m_name         = j["name"].get<std::string_view>();
    object.m_geometry_ids = j["geometries"].get<vector<IndexT>>();
}
}  // namespace uipc::core

namespace fmt
{
appender formatter<uipc::core::Object>::format(const uipc::core::Object& c,
                                               format_context& ctx) const
{
    fmt::format_to(ctx.out(), "Object[{}(#{})]:", c.name(), c.id());
    for(auto id : c.geometries().ids())
    {
        auto geo_slot      = c.m_scene->geometries().find(id);
        auto rest_geo_slot = c.m_scene->rest_geometries().find(id);

        fmt::format_to(ctx.out(),
                       "\n  [{}] <{}, {}>",
                       id,
                       geo_slot->geometry().type(),
                       rest_geo_slot->geometry().type());
    }

    return ctx.out();
}
}  // namespace fmt
