#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/world/scene.h>

namespace uipc::backend
{
SceneVisitor::SceneVisitor(world::Scene& scene) noexcept
    : m_scene(scene)
{
}

void SceneVisitor::begin_pending() noexcept
{
    m_scene.m_impl.started = true;
}

void SceneVisitor::solve_pending() noexcept
{
    m_scene.solve_pending();
}

span<S<geometry::GeometrySlot>> SceneVisitor::geometries() const noexcept
{
    return m_scene.m_impl.geometries.geometry_slots();
}

span<S<geometry::GeometrySlot>> SceneVisitor::pending_geometries() const noexcept
{
    return m_scene.m_impl.geometries.pending_create_slots();
}

span<S<geometry::GeometrySlot>> SceneVisitor::rest_geometries() const noexcept
{
    return m_scene.m_impl.rest_geometries.geometry_slots();
}

span<S<geometry::GeometrySlot>> SceneVisitor::pending_rest_geometries() const noexcept
{
    return m_scene.m_impl.rest_geometries.pending_create_slots();
}

span<IndexT> SceneVisitor::pending_destroy_ids() const noexcept
{
    return m_scene.m_impl.geometries.pending_destroy_ids();
}
const Json& SceneVisitor::info() const noexcept
{
    return m_scene.info();
}
const world::ConstitutionTabular& SceneVisitor::constitution_tabular() const noexcept
{
    return m_scene.constitution_tabular();
}
const world::ContactTabular& SceneVisitor::contact_tabular() const noexcept
{
    return m_scene.contact_tabular();
}
}  // namespace uipc::backend
