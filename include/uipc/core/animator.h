#pragma once
#include <uipc/core/object.h>
#include <uipc/core/animation.h>
#include <functional>
#include <uipc/common/unordered_map.h>

namespace uipc::backend
{
class AnimatorVisitor;
}

namespace uipc::core
{
class Scene;
class UIPC_CORE_API Animator
{
  public:
    void  substep(SizeT n) noexcept;
    SizeT substep() const noexcept;

    void insert(Object& obj, Animation::ActionOnUpdate&& on_update);
    void erase(IndexT id);

    // delete copy/move constructor/assignment
    Animator(const Animator&)            = delete;
    Animator(Animator&&)                 = delete;
    Animator& operator=(const Animator&) = delete;
    Animator& operator=(Animator&&)      = delete;

  private:
    friend class Scene;
    friend class backend::AnimatorVisitor;

    Animator(Scene& scene) noexcept;  // only called by Scene

    unordered_map<IndexT, Animation> m_animations;
    Scene&                           m_scene;
    SizeT                            m_substep = 1;
};
}  // namespace uipc::core
