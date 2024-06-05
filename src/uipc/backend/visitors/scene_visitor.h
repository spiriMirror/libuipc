#pragma once
#include <uipc/common/type_define.h>
#include <uipc/world/object.h>
#include <uipc/common/set.h>
#include <uipc/common/unordered_map.h>
#include <uipc/geometry/geometry_collection.h>

namespace uipc::world
{
class Scene;
}

namespace uipc::backend
{
class UIPC_CORE_API SceneVisitor
{
  public:
    SceneVisitor(world::Scene& scene) noexcept;
    void begin_pending() noexcept;
    void solve_pending() noexcept;

    span<P<geometry::GeometrySlot>> geometries() const noexcept;
    span<P<geometry::GeometrySlot>> pending_geometries() const noexcept;


    span<P<geometry::GeometrySlot>> rest_geometries() const noexcept;
    span<P<geometry::GeometrySlot>> pending_rest_geometries() const noexcept;

    span<IndexT> pending_destroy_ids() const noexcept;

  private:
    world::Scene& m_scene;
};
}  // namespace uipc::backend
