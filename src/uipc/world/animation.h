#pragma once
#include <uipc/common/type_define.h>
#include <uipc/world/object.h>
#include <uipc/common/span.h>
#include <functional>
namespace uipc::world
{
class World;
class Animation
{
  public:
    class UpdateHint
    {
      public:
        void fixed_vertices_changing(bool v) noexcept;

      private:
        friend class Animation;
        bool m_fixed_vertices_changing = false;
    };


    class UpdateInfo
    {
      public:
        Object&                         object() const noexcept;
        span<S<geometry::GeometrySlot>> geo_slots() const noexcept;
        span<S<geometry::GeometrySlot>> rest_geo_slots() const noexcept;
        UpdateHint&                     hint() noexcept;

      private:
        UpdateInfo(Animation& animation) noexcept;
        friend class Animation;
        Animation* m_animation = nullptr;
        UpdateHint m_hint;
    };

    using ActionOnUpdate = std::function<void(UpdateInfo&)>;

  private:
    friend class Animator;
    friend class AnimatorVisitor;
    void init();
    void update();

    Animation(World& world, Object& object, ActionOnUpdate&& on_update) noexcept;

    Object*        m_object = nullptr;
    World*         m_world  = nullptr;
    ActionOnUpdate m_on_update;

    mutable vector<S<geometry::GeometrySlot>> m_temp_geo_slots;
    mutable vector<S<geometry::GeometrySlot>> m_temp_rest_geo_slots;
};
}  // namespace uipc::world
