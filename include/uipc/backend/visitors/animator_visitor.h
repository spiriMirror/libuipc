#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/macro.h>

namespace uipc::core
{
class Animator;
}

namespace uipc::backend
{
class UIPC_CORE_API AnimatorVisitor
{
  public:
    AnimatorVisitor(core::Animator& animator) noexcept;
    void  init();
    void  update();
    SizeT substep() noexcept;

  private:
    core::Animator& m_animator;
};
}  // namespace uipc::backend
