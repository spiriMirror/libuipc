#pragma once
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/animator_visitor.h>
namespace uipc::core
{
class World;
class Engine;
}  // namespace uipc::core

namespace uipc::core::internal
{
class World;
class Scene;
class Engine;
}  // namespace uipc::core::internal

namespace uipc::backend
{
class UIPC_CORE_API WorldVisitor
{
  public:
    WorldVisitor(core::World& w) noexcept;
    WorldVisitor(core::internal::World& w) noexcept;

    WorldVisitor(const WorldVisitor&)            = delete;
    WorldVisitor(WorldVisitor&&)                 = default;
    WorldVisitor& operator=(const WorldVisitor&) = delete;
    WorldVisitor& operator=(WorldVisitor&&)      = default;

    SceneVisitor    scene() noexcept;
    AnimatorVisitor animator() noexcept;
    core::World&    ref() noexcept;

  private:
    mutable S<core::World> m_ref;
    core::internal::World& m_world;
};
}  // namespace uipc::backend
