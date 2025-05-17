#include <uipc/core/animation.h>
#include <uipc/core/world.h>
#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/implicit_geometry.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/core/internal/scene.h>

namespace uipc::core
{
Animation::Animation(internal::Scene& scene, Object& object, ActionOnUpdate&& on_update) noexcept
    : m_scene(&scene)
    , m_object(&object)
    , m_on_update(std::move(on_update))
{
}

void Animation::init()
{
    auto scene   = backend::SceneVisitor(*m_scene);
    auto geo_ids = m_object->geometries().ids();
    for(auto id : geo_ids)
    {
        auto slot = scene.find_geometry(id);
        UIPC_ASSERT(slot, "Animation: Geometry slot not found for id={}", id);
        m_temp_geo_slots.push_back(slot);
        auto rest_slot = scene.find_rest_geometry(id);
        UIPC_ASSERT(rest_slot, "Animation: Rest geometry slot not found for id={}", id);
        m_temp_rest_geo_slots.push_back(rest_slot);
    }
}

void Animation::update()
{
    UpdateInfo info{*this};
    m_on_update(info);
}

Float Animation::UpdateInfo::dt() const noexcept
{
    return m_animation->m_scene->dt();
}

Object& Animation::UpdateInfo::object() const noexcept
{
    return *(m_animation->m_object);
}

span<S<geometry::GeometrySlot>> Animation::UpdateInfo::geo_slots() const noexcept
{
    return m_animation->m_temp_geo_slots;
}

span<S<geometry::GeometrySlot>> Animation::UpdateInfo::rest_geo_slots() const noexcept
{
    return m_animation->m_temp_rest_geo_slots;
}

SizeT Animation::UpdateInfo::frame() const noexcept
{
    return m_animation->m_scene->world()->frame();
}

auto Animation::UpdateInfo::hint() noexcept -> UpdateHint&
{
    return m_hint;
}

Animation::UpdateInfo::UpdateInfo(Animation& animation) noexcept
    : m_animation(&animation)
{
}

void Animation::UpdateHint::fixed_vertices_changing(bool v) noexcept
{
    m_fixed_vertices_changing = v;
}
}  // namespace uipc::core

namespace fmt
{
appender fmt::formatter<uipc::core::Animation>::format(const uipc::core::Animation& c,
                                                       format_context& ctx) const
{
    return fmt::format_to(ctx.out(), "Animation[{}]", *c.m_object);
}
}  // namespace fmt