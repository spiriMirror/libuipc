#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/world/scene.h>

namespace uipc::backend
{
SceneVisitor::SceneVisitor(world::Scene& scene) noexcept
    : m_scene(scene)
{
}

void SceneVisitor::solve_pending() noexcept
{
    m_scene.solve_pending();
}
}  // namespace uipc::backend
