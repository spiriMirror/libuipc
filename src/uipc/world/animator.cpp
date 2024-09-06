#include <uipc/world/animator.h>
#include <uipc/common/log.h>
namespace uipc::world
{
void Animator::insert(Object& obj, Animation::ActionOnUpdate&& on_update)
{
    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto it = m_animations.find(obj.id());

        if(it != m_animations.end())
        {
            auto obj = it->second.m_object;
            UIPC_ASSERT(it == m_animations.end(),
                        "Animator: Object (name={}, id={}) already has an animation.",
                        obj->name(),
                        obj->id());
        }
    }

    m_animations.emplace(obj.id(), Animation(*m_world, obj, std::move(on_update)));
}

void Animator::erase(IndexT id)
{
    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto it = m_animations.find(id);
        if(it == m_animations.end())
        {
            UIPC_WARN_WITH_LOCATION("Animator: No animation found for object id={}, ignore.", id);
        }
    }
    m_animations.erase(id);
}

Animator::Animator(World& world)
    : m_world(&world)
{
}
}  // namespace uipc::world
