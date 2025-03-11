#pragma once
#pragma once
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/animator_visitor.h>
namespace uipc::core
{
class World;
class Engine;
}  // namespace uipc::core

namespace uipc::backend
{
class UIPC_CORE_API WorldVisitor
{
  public:
    WorldVisitor(core::World& w) noexcept;
    SceneVisitor    scene() noexcept;
    AnimatorVisitor animator() noexcept;
    core::World&    ref() noexcept;

  private:
    core::World& m_world;
};
}  // namespace uipc::backend
