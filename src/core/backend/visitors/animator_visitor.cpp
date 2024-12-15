#include <uipc/backend/visitors/animator_visitor.h>
#include <uipc/core/animator.h>

namespace uipc::backend
{
AnimatorVisitor::AnimatorVisitor(core::Animator& animator) noexcept
    : m_animator(animator)
{
}

void AnimatorVisitor::init()
{
    for(auto& [id, animation] : m_animator.m_animations)
    {
        animation.init();
    }
}
void AnimatorVisitor::update()
{
    for(auto& [id, animation] : m_animator.m_animations)
    {
        animation.update();
    }
}
SizeT AnimatorVisitor::substep() noexcept
{
    return m_animator.substep();
}
}  // namespace uipc::backend
