#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>

namespace uipc::backend
{
WorldVisitor::WorldVisitor(core::World& w) noexcept
    : m_world(w)
{
}

SceneVisitor WorldVisitor::scene() noexcept
{
    return SceneVisitor{*m_world.m_scene};
}

AnimatorVisitor WorldVisitor::animator() noexcept
{
    return AnimatorVisitor{m_world.m_scene->animator()};
}
}  // namespace uipc::backend
