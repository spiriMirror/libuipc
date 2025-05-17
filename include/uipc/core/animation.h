#pragma once
#include <uipc/common/type_define.h>
#include <uipc/core/object.h>
#include <uipc/common/span.h>
#include <functional>

namespace uipc::backend
{
class AnimatorVisitor;
}

namespace uipc::core
{
class World;
class UIPC_CORE_API Animation
{
  public:
    class UIPC_CORE_API UpdateHint
    {
      public:
        void fixed_vertices_changing(bool v) noexcept;

      private:
        friend class Animation;
        bool m_fixed_vertices_changing = false;
    };


    class UIPC_CORE_API UpdateInfo
    {
      public:
        Float                           dt() const noexcept;
        Object&                         object() const noexcept;
        span<S<geometry::GeometrySlot>> geo_slots() const noexcept;
        span<S<geometry::GeometrySlot>> rest_geo_slots() const noexcept;
        SizeT                           frame() const noexcept;
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
    friend class backend::AnimatorVisitor;
    friend struct fmt::formatter<Animation>;
    void init();
    void update();

    Animation(internal::Scene& scene, Object& object, ActionOnUpdate&& on_update) noexcept;

    Object*          m_object = nullptr;
    internal::Scene* m_scene  = nullptr;
    ActionOnUpdate   m_on_update;

    mutable vector<S<geometry::GeometrySlot>> m_temp_geo_slots;
    mutable vector<S<geometry::GeometrySlot>> m_temp_rest_geo_slots;
};
}  // namespace uipc::core

namespace fmt
{
template <>
struct UIPC_CORE_API formatter<uipc::core::Animation> : formatter<string_view>
{
    appender format(const uipc::core::Animation& c, format_context& ctx) const;
};
}  // namespace fmt
