#pragma once

#include <uipc/world/animator.h>

namespace uipc::backend
{
class UIPC_CORE_API AnimatorVisitor
{
  public:
    AnimatorVisitor(world::Animator& animator) noexcept;
    void init();
    void update();

  private:
    world::Animator& m_animator;
};
}  // namespace uipc::backend
