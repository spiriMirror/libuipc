#pragma once
#include <uipc/world/object_slot.h>
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
    const set<IndexT>& pending_destroy() noexcept;
    const unordered_map<IndexT, S<world::ObjectSlot>>& pending_create() noexcept;
    void solve_pending() noexcept;

  private:
    world::Scene& m_scene;
};
}  // namespace uipc::backend
