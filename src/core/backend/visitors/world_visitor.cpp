#include <uipc/backend/visitors/world_visitor.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>
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
}

SceneVisitor WorldVisitor::scene() noexcept
{
    return SceneVisitor{*m_world.m_scene};
}

AnimatorVisitor WorldVisitor::animator() noexcept
{
    return AnimatorVisitor{m_world.m_scene->animator()};
}

core::Engine WorldVisitor::engine() const noexcept
{
    return core::Engine{m_world.m_engine.lock()};
}

core::World WorldVisitor::get() const noexcept
{
    return core::World{m_world.shared_from_this()};
}
}  // namespace uipc::backend
