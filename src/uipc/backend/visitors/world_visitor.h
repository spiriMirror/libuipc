#pragma once
#include <uipc/backend/visitors/scene_visitor.h>

namespace uipc::world
{
class World;
}

namespace uipc::backend
{
class UIPC_CORE_API WorldVisitor
{
  public:
    WorldVisitor(world::World& w) noexcept;
    SceneVisitor scene() noexcept;

  private:
    world::World& m_world;
};
}  // namespace uipc::backend
