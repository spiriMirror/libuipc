#pragma once
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/animator_visitor.h>
namespace uipc::core
{
class World;
}

namespace uipc::backend
{
class UIPC_CORE_API WorldVisitor
{
  public:
    WorldVisitor(core::World& w) noexcept;
    SceneVisitor    scene() noexcept;
    AnimatorVisitor animator() noexcept;

  private:
    core::World& m_world;
};
}  // namespace uipc::backend
