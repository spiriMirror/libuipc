#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/internal/world.h>

namespace uipc::backend
{
WorldVisitor::WorldVisitor(core::World& w) noexcept
    : m_world(*w.m_internal)
{
}

WorldVisitor::WorldVisitor(core::internal::World& w) noexcept
    : m_world(w)
{
    m_ref = S<core::World>{new core::World(w.shared_from_this())};
}

SceneVisitor WorldVisitor::scene() noexcept
{
    return SceneVisitor{*m_world.m_scene};
}

AnimatorVisitor WorldVisitor::animator() noexcept
{
    return AnimatorVisitor{m_world.m_scene->animator()};
}

core::World& WorldVisitor::ref() noexcept
{
    return *m_ref;
}
}  // namespace uipc::backend
