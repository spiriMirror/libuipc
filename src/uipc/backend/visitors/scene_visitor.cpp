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
    m_scene.m_started = true;
}

void SceneVisitor::solve_pending() noexcept
{
    m_scene.solve_pending();
}

span<P<geometry::GeometrySlot>> SceneVisitor::geometries() const noexcept
{
    return m_scene.m_geometries.geometry_slots();
}

span<P<geometry::GeometrySlot>> SceneVisitor::pending_geometries() const noexcept
{
    return m_scene.m_geometries.pending_create_slots();
}

span<P<geometry::GeometrySlot>> SceneVisitor::rest_geometries() const noexcept
{
    return m_scene.m_rest_geometries.geometry_slots();
}

span<P<geometry::GeometrySlot>> SceneVisitor::pending_rest_geometries() const noexcept
{
    return m_scene.m_rest_geometries.pending_create_slots();
}

span<IndexT> SceneVisitor::pending_destroy_ids() const noexcept
{
    return m_scene.m_geometries.pending_destroy_ids();
}
}  // namespace uipc::backend
