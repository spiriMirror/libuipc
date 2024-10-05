#pragma once

#include <uipc/core/animator.h>

namespace uipc::backend
{
class UIPC_CORE_API AnimatorVisitor
{
  public:
    AnimatorVisitor(core::Animator& animator) noexcept;
    void init();
    void update();

  private:
    core::Animator& m_animator;
};
}  // namespace uipc::backend
