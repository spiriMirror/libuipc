#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/world/scene.h>

namespace uipc::backend
{
SceneVisitor::SceneVisitor(world::Scene& scene) noexcept
    : m_scene(scene)
{
}

const set<IndexT>& SceneVisitor::pending_destroy() noexcept
{
    return m_scene.pending_destroy();
}

const unordered_map<IndexT, S<world::ObjectSlot>>& SceneVisitor::pending_create() noexcept
{
    return m_scene.pending_create();
}

void SceneVisitor::solve_pending() noexcept
{
    m_scene.solve_pending();
}
}  // namespace uipc::backend
