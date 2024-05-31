#include <uipc/world/object_slot.h>

namespace uipc::world
{
IndexT IObjectSlot::id() const noexcept
{
    return get_id();
}

ObjectSlot::ObjectSlot(IndexT id, std::string_view name) noexcept
    : m_id{id}
    , m_object{name}
{
}

ObjectSlot::ObjectSlot(IndexT id, Object&& object) noexcept
    : m_id{id}
    , m_object{std::move(object)}
{
}

Object::CGeometries ObjectSlot::geometries() const
{
    return m_object.geometries();
}

Object::Geometries ObjectSlot::geometries()
{
    return m_object.geometries();
}

IndexT ObjectSlot::get_id() const noexcept
{
    return m_id;
}
}  // namespace uipc::world
