#pragma once
#include <uipc/world/object.h>
#include <uipc/world/animation.h>
#include <functional>
#include <uipc/common/unordered_map.h>

namespace uipc::backend
{
class AnimatorVisitor;
}

namespace uipc::world
{
class World;
class Animator
{
  public:
    void insert(Object& obj, Animation::ActionOnUpdate&& on_update);
    void erase(IndexT id);

    // delete copy/move constructor/assignment
    Animator(const Animator&)            = delete;
    Animator(Animator&&)                 = delete;
    Animator& operator=(const Animator&) = delete;
    Animator& operator=(Animator&&)      = delete;

  private:
    friend class World;
    friend class backend::AnimatorVisitor;

    Animator(World& world);  // only called by World

    unordered_map<IndexT, Animation> m_animations;
    World*                           m_world = nullptr;
};
}  // namespace uipc::world
