#pragma once
#include <uipc/common/type_define.h>
#include <uipc/world/object.h>
#include <uipc/common/set.h>
#include <uipc/common/unordered_map.h>

namespace uipc::world
{
class Scene;
}

namespace uipc::backend
{
class SceneVisitor
{
  public:
    SceneVisitor(world::Scene& scene) noexcept;
    void solve_pending() noexcept;

  private:
    world::Scene& m_scene;
};
}  // namespace uipc::backend
